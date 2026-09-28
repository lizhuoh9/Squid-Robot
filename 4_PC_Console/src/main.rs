//! Squid Robot PC 控制台。
//!
//! PC ──USB 串口(115200)── Minima HC-12 Bridge ──HC-12 无线(9600)── ESP32 机器人
//!
//! 用法：
//!   squid-console                                          自动发现 Bridge（发 ?ID 识别帧）
//!   squid-console [--port COM9] [--baud 115200]            指定串口
//!   squid-console --list                                   列出串口（带 Bridge 标记）
//!   squid-console --port COM9 --script "3:g,5:w,15:w" --duration 20
//!       无界面模式：按 "秒:命令" 定时发送，结束时把仪表盘渲染成文本打印（用于测试/远程检查）

mod app;
mod commands;
mod link;
mod parser;
mod picker;
mod ui;

use std::fs::{self, File};
use std::io;
use std::sync::mpsc::TryRecvError;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use ratatui::backend::TestBackend;
use ratatui::crossterm::event::{
    self, DisableMouseCapture, EnableMouseCapture, Event, KeyEventKind,
};
use ratatui::crossterm::execute;
use ratatui::Terminal;
use unicode_width::UnicodeWidthStr;

use app::{Action, App, LogKind};
use link::{Link, RxMsg};

/// 心跳周期：刷新机器人端的遥控失联保护计时（机器人默认 20s 无命令即急停）。
const HEARTBEAT_SECS: u64 = 5;

struct Args {
    port: Option<String>,
    baud: u32,
    list: bool,
    script: Option<Vec<(f32, String)>>,
    duration: f32,
    width: u16,
    height: u16,
    no_log: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut a = Args {
        port: None,
        baud: 115_200,
        list: false,
        script: None,
        duration: 20.0,
        width: 120,
        height: 40,
        no_log: false,
    };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        let mut val = |name: &str| it.next().ok_or_else(|| format!("{name} 값이 필요합니다"));
        match k.as_str() {
            "--port" | "-p" => a.port = Some(val("--port")?),
            "--baud" | "-b" => a.baud = val("--baud")?.parse().map_err(|_| "--baud는 숫자여야 합니다")?,
            "--list" | "-l" => a.list = true,
            "--duration" => a.duration = val("--duration")?.parse().map_err(|_| "--duration은 숫자여야 합니다")?,
            "--size" => {
                let v = val("--size")?;
                let (w, h) = v.split_once('x').ok_or("--size 형식은 120x40입니다")?;
                a.width = w.parse().map_err(|_| "--size 너비가 잘못되었습니다")?;
                a.height = h.parse().map_err(|_| "--size 높이가 잘못되었습니다")?;
            }
            "--no-log" => a.no_log = true,
            "--script" => {
                let v = val("--script")?;
                let mut steps = Vec::new();
                for item in v.split(',').filter(|s| !s.trim().is_empty()) {
                    let (t, cmd) = item.split_once(':').ok_or(format!("스크립트 항목 {item}은 초:명령 형식이어야 합니다"))?;
                    let t: f32 = t.trim().parse().map_err(|_| format!("스크립트 시간 {t}은 숫자가 아닙니다"))?;
                    steps.push((t, cmd.trim().to_string()));
                }
                steps.sort_by(|x, y| x.0.total_cmp(&y.0));
                a.script = Some(steps);
            }
            "--help" | "-h" => {
                println!(
                    "squid-console [--port COM9] [--baud 115200] [--list] [--no-log]\n\
                     --port를 생략하면 Bridge를 자동 검색합니다. F3로 화면에서 포트를 변경할 수 있습니다\n\
                     헤드리스 테스트: --script \"3:g,5:w,15:w\" --duration 20 [--size 120x40]"
                );
                std::process::exit(0);
            }
            other => return Err(format!("알 수 없는 인자 {other} (--help로 사용법 확인)")),
        }
    }
    Ok(a)
}

fn open_raw_log(no_log: bool) -> (Option<File>, Option<String>) {
    if no_log {
        return (None, None);
    }
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let _ = fs::create_dir_all("squid-logs");
    let path = format!("squid-logs/raw-{secs}.log");
    match File::create(&path) {
        Ok(f) => (Some(f), Some(path)),
        Err(_) => (None, None),
    }
}

fn drain(app: &mut App, link: &Link) {
    loop {
        match link.rx.try_recv() {
            Ok(RxMsg::Line(l)) => app.on_line(l),
            Ok(RxMsg::Closed(e)) => {
                app.push_log(LogKind::Local, format!("시리얼 포트 연결 끊김: {e}"));
                app.link_error = Some(e);
            }
            Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break,
        }
    }
}

fn send(app: &mut App, link: &mut Link, cmd: &str) {
    match link.send(cmd) {
        Ok(()) => app.on_sent(cmd),
        Err(e) => app.push_log(LogKind::Local, format!("전송 실패 {cmd}: {e}")),
    }
}

/// 心跳：不写日志，只计数；机器人收到任何命令都会刷新失联计时，hb 只是保证"有命令"。
fn heartbeat(app: &mut App, link: &mut Link) {
    if app.last_hb.elapsed() < Duration::from_secs(HEARTBEAT_SECS) {
        return;
    }
    app.last_hb = Instant::now();
    if link.send("hb").is_ok() {
        app.hb_count += 1;
    }
}

enum Exit {
    Quit,
    Reselect,
}

fn run_tui(terminal: &mut ratatui::DefaultTerminal, app: &mut App, link: &mut Link) -> io::Result<Exit> {
    loop {
        drain(app, link);
        heartbeat(app, link);

        // 一帧内把排队的按键全处理掉：按键先发命令，再统一重绘。
        // （以前一帧只吃一个按键，连按时会明显滞后。）
        while event::poll(Duration::ZERO)? {
            match event::read()? {
                // Windows 终端按下/松开各报一次，只处理按下
                Event::Key(key) if key.kind == KeyEventKind::Press => match app.on_key(key) {
                    Action::Quit => return Ok(Exit::Quit),
                    Action::Reselect => return Ok(Exit::Reselect),
                    Action::Send(cmd) => send(app, link, &cmd),
                    Action::None => {}
                },
                // 滚轮/点击一律忽略：定深只认键盘，免得手一滑就发指令
                Event::Mouse(_) => {}
                _ => {}
            }
            drain(app, link);
        }

        terminal.draw(|f| ui::draw(f, app))?;
        // 有按键或到点就醒，回传数据最迟 30ms 上屏
        let _ = event::poll(Duration::from_millis(30))?;
    }
}

/// 无界面：按脚本发命令，结束时把仪表盘渲染成纯文本打印。
fn run_headless(mut app: App, mut link: Link, script: Vec<(f32, String)>, duration: f32, w: u16, h: u16) {
    let start = Instant::now();
    let mut next = 0;
    while start.elapsed().as_secs_f32() < duration {
        let t = start.elapsed().as_secs_f32();
        while next < script.len() && script[next].0 <= t {
            let cmd = script[next].1.clone();
            send(&mut app, &mut link, &cmd);
            next += 1;
        }
        drain(&mut app, &link);
        heartbeat(&mut app, &mut link);
        std::thread::sleep(Duration::from_millis(20));
    }
    drain(&mut app, &link);

    let mut term = Terminal::new(TestBackend::new(w, h)).expect("TestBackend");
    term.draw(|f| ui::draw(f, &app)).expect("draw");
    let buf = term.backend().buffer();
    for y in 0..h {
        let mut row = String::new();
        let mut x = 0;
        while x < w {
            let sym = buf[(x, y)].symbol();
            row.push_str(sym);
            x += (sym.width() as u16).max(1);
        }
        println!("{}", row.trim_end());
    }
    // 命令往返延迟（发出 → 机器人回 [OK]/Bridge 回 [ACK]）
    let mut r = app.rtt_ms.clone();
    r.sort_unstable();
    if !r.is_empty() {
        let sum: u32 = r.iter().sum();
        println!(
            "
[지연 시간] 샘플={} 최소={}ms 중앙={}ms 평균={}ms 최대={}ms 전체={:?}",
            r.len(), r[0], r[r.len() / 2], sum / r.len() as u32, r[r.len() - 1], r
        );
    }
    // 面板到达间隔：机器人固定 1s 一帧，抖动反映主循环被卡多久
    let mut g = app.panel_gaps_ms.clone();
    g.sort_unstable();
    if g.len() > 2 {
        println!(
            "[패널 간격] 프레임={} 최소={}ms 중앙={}ms P90={}ms 최대={}ms",
            g.len() + 1, g[0], g[g.len() / 2], g[g.len() * 9 / 10], g[g.len() - 1]
        );
    }

    let s = &app.state;
    println!(
        "\n[요약] 전송={} 패널={} 로봇[OK]={} BridgeACK={} 재시도={} 실패={} 로그 줄={}",
        app.tx_count, s.panels, s.robot_oks, s.acks, s.retries, s.noacks, app.log.len()
    );
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    if args.list {
        println!("검색 중 (?ID 식별 프레임을 각 시리얼 포트로 전송)…");
        for c in link::discover(true) {
            println!("{:<6} {}", c.name, c.label);
        }
        return;
    }

    // 无界面脚本模式：必须给定串口，或自动发现到唯一 Bridge。
    if let Some(script) = args.script {
        let port = args.port.clone().or_else(|| {
            let found = link::discover(true);
            let bridges: Vec<_> = found.iter().filter(|c| c.is_bridge).collect();
            (bridges.len() == 1).then(|| bridges[0].name.clone())
        });
        let Some(port) = port else {
            eprintln!("Bridge를 자동으로 찾지 못했습니다. --port로 시리얼 포트를 지정하세요.");
            std::process::exit(1);
        };
        let (raw_file, raw_path) = open_raw_log(args.no_log);
        match Link::open(&port, args.baud, raw_file) {
            Ok(link) => {
                let app = App::new(port, args.baud, raw_path);
                run_headless(app, link, script, args.duration, args.width, args.height);
            }
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        }
        return;
    }

    // 交互模式：自动发现 → 仪表盘；F3 可回到选择界面换串口。
    let mut terminal = ratatui::init();
    // 抓住鼠标事件。不抓的话，终端会把滚轮翻译成 Up/Down 或 PgUp/PgDn 按键，
    // 而那几个键是定深滑块 —— 随手滚一下就往机器人发 l<n>，浮沉泵真的会动。
    // 抓了之后滚轮走 Event::Mouse，在事件循环里被直接忽略。
    // 代价：终端选中文字要按住 Shift（标准做法）。
    let _ = execute!(io::stdout(), EnableMouseCapture);
    let mut forced = args.port.clone();
    let result = (|| -> io::Result<()> {
        loop {
            let port = match forced.take() {
                Some(p) => p,
                None => match picker::pick(&mut terminal, true)? {
                    picker::Pick::Port(p) => p,
                    picker::Pick::Quit => return Ok(()),
                },
            };
            let (raw_file, raw_path) = open_raw_log(args.no_log);
            let mut app = App::new(port.clone(), args.baud, raw_path);
            let mut link = match Link::open(&port, args.baud, raw_file) {
                Ok(l) => l,
                Err(e) => {
                    // 打不开就回到选择界面，并把原因显示出来
                    app.link_error = Some(e.to_string());
                    match picker::pick(&mut terminal, false)? {
                        picker::Pick::Port(p2) => {
                            forced = Some(p2);
                            continue;
                        }
                        picker::Pick::Quit => return Ok(()),
                    }
                }
            };
            match run_tui(&mut terminal, &mut app, &mut link)? {
                Exit::Quit => return Ok(()),
                Exit::Reselect => continue,
            }
        }
    })();
    let _ = execute!(io::stdout(), DisableMouseCapture);
    ratatui::restore();
    if let Err(e) = result {
        eprintln!("화면 오류: {e}");
        std::process::exit(1);
    }
}
