//! 应用状态：机器人状态 + 日志 + 深度滑块 + 按键映射。
//!
//! 按键哲学：**所有运动键直接实时发送**，不需要先切模式。需要打字的命令
//! （mark/mt/md/pset/fs/信道…）按**回车**打开底部输入框，输入期间字母就是字母；
//! 急停键（Esc/空格/s）在任何时候都直达机器人。

use std::collections::VecDeque;
use std::fs::{self, File};
use std::io::Write;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use ratatui::crossterm::event::{KeyCode, KeyEvent, KeyModifiers};

use crate::parser::{LineKind, RobotState};

const LOG_MAX: usize = 2000;

/// 深度滑块：0~100cm，每格 5cm；0 = 上浮到水面。
pub const DEPTH_STEP_CM: u16 = 5;
pub const DEPTH_MAX_CM: u16 = 100;

pub struct LogLine {
    pub t: f32,
    pub kind: LogKind,
    pub text: String,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum LogKind {
    Rx(LineKind),
    Tx,
    Local,
}

/// 按键处理结果：主循环据此发送命令或退出。
pub enum Action {
    None,
    Send(String),
    Quit,
    /// 回到串口选择界面（F3）
    Reselect,
}

pub struct App {
    pub state: RobotState,
    pub log: VecDeque<LogLine>,
    /// ":" 打开的命令输入；None = 未打开（此时所有按键直发）
    pub input: Option<String>,
    pub show_panel_lines: bool,
    pub port: String,
    pub baud: u32,
    pub start: Instant,
    pub tx_count: u32,
    pub link_error: Option<String>,
    pub raw_log_path: Option<String>,
    pub control_log: Option<File>,
    control_logging_allowed: bool,
    pending_control_commands: Vec<String>,
    pub last_cmd: Option<String>,
    pub hb_count: u32,
    pub last_hb: Instant,
    /// 深度滑块当前刻度（cm）。PgUp/PgDn 每次动一格并立即下发。
    pub depth_slider_cm: u16,
    /// 右侧命令栏滚动位置（F5/F6）
    pub side_scroll: u16,
    /// 最近一条待确认的命令（用于量往返延迟）
    pending: Option<(String, Instant)>,
    /// 命令往返延迟样本（ms）：发出 → 机器人回 [OK]<cmd>
    pub rtt_ms: Vec<u32>,
    pub rtt_last: Option<u32>,
    /// 面板帧到达间隔（ms）：机器人固定 1s 打一帧，抖动=主循环被什么卡住了
    pub panel_gaps_ms: Vec<u32>,
    last_panel: Option<Instant>,
}

impl App {
    pub fn new(port: String, baud: u32, raw_log_path: Option<String>) -> Self {
        let control_logging_allowed = raw_log_path.is_some();

        Self {
            state: RobotState::new(),
            log: VecDeque::new(),
            input: None,
            show_panel_lines: false,
            port,
            baud,
            start: Instant::now(),
            tx_count: 0,
            link_error: None,
            raw_log_path,
            control_log: None,
            control_logging_allowed,
            pending_control_commands: Vec::new(),
            last_cmd: None,
            hb_count: 0,
            last_hb: Instant::now(),
            depth_slider_cm: 0,
            side_scroll: 0,
            pending: None,
            rtt_ms: Vec::new(),
            rtt_last: None,
            panel_gaps_ms: Vec::new(),
            last_panel: None,
        }
    }

    fn elapsed(&self) -> f32 {
        self.start.elapsed().as_secs_f32()
    }

    pub fn push_log(&mut self, kind: LogKind, text: impl Into<String>) {
        if self.log.len() == LOG_MAX {
            self.log.pop_front();
        }
        let t = self.elapsed();
        self.log.push_back(LogLine { t, kind, text: text.into() });
    }

    pub fn on_line(&mut self, line: String) {
        let now = Instant::now();
        // 往返延迟：机器人收到命令会原样回 "[OK]<cmd>"（Bridge 可能把它劈成两行）
        if let Some((cmd, sent)) = self.pending.clone() {
            let t = line.trim();
            if t == format!("[OK]{cmd}") || t == format!("[OK] {cmd}") || t == format!("[ACK] {cmd}") {
                let ms = sent.elapsed().as_millis() as u32;
                self.rtt_last = Some(ms);
                self.rtt_ms.push(ms);
                self.pending = None;
            }
        }
        // 面板头一行当作帧起点，量到达间隔
        if line.contains("ALL SENSORS") {
            if let Some(prev) = self.last_panel {
                self.panel_gaps_ms.push(now.duration_since(prev).as_millis() as u32);
            }
            self.last_panel = Some(now);
        }
        let is_control = line.trim().starts_with("[CTRL]");
        let kind = self.state.parse_line(&line, now);
        if is_control {
            self.write_control_sample();
        }
        // 机器人确认的目标深度回来了：滑块跟着走（别人/别的来源改过也同步）
        if let Some(t) = self.state.target_depth {
            let snapped = ((t / DEPTH_STEP_CM as f32).round() as u16) * DEPTH_STEP_CM;
            self.depth_slider_cm = snapped.min(DEPTH_MAX_CM);
        }
        if !line.trim().is_empty() {
            self.push_log(LogKind::Rx(kind), localize_rx_line(&line));
        }
    }

    pub fn on_sent(&mut self, cmd: &str) {
        self.pending = Some((cmd.to_string(), Instant::now()));
        self.tx_count += 1;
        self.last_cmd = Some(cmd.to_string());
        if self.control_log.is_some() {
            self.pending_control_commands.push(cmd.to_string());
        }
        self.push_log(LogKind::Tx, format!("> {cmd}"));
    }

    fn toggle_control_logging(&mut self) {
        if !self.control_logging_allowed {
            self.push_log(LogKind::Local, "제어 CSV 저장이 비활성화되어 있습니다 (--no-log)");
            return;
        }
        if self.control_log.is_some() {
            if !self.pending_control_commands.is_empty() {
                self.write_control_sample();
            }
            if let Some(mut file) = self.control_log.take() {
                let _ = file.flush();
            }
            self.pending_control_commands.clear();
            self.push_log(LogKind::Local, "제어 CSV 저장 종료");
            return;
        }

        let secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let _ = fs::create_dir_all("squid-logs");
        let path = format!("squid-logs/control-{secs}.csv");
        let mut file = match File::create(&path) {
            Ok(file) => file,
            Err(e) => {
                self.push_log(LogKind::Local, format!("제어 CSV 저장 시작 실패: {e}"));
                return;
            }
        };
        if file
            .write_all(
                b"elapsed_s,depth_cm,depth_speed_cm_s,depth_accel_cm_s2,target_depth_cm,pid_output,pid_base,buoyancy_pwm,valve_a_on,valve_b_on,valve_e_on,valve_f_on,commands,forward_active\r\n",
            )
            .is_err()
        {
            self.push_log(LogKind::Local, "제어 CSV 헤더 쓰기 실패");
            return;
        }
        self.control_log = Some(file);
        self.pending_control_commands.clear();
        self.push_log(LogKind::Local, format!("제어 CSV 저장 시작: {path}"));
    }

    fn write_control_sample(&mut self) {
        if self.control_log.is_none() {
            return;
        }
        let mask = self.state.actuator_mask;
        let bit = |b: u16| mask.map(|m| if m & b != 0 { 1 } else { 0 });
        let opt = |v: Option<f32>| v.map(|x| format!("{x:.3}")).unwrap_or_default();
        let commands = self.pending_control_commands.drain(..).collect::<Vec<_>>().join("|");
        let commands = if commands.contains([',', '"', '\r', '\n']) {
            format!("\"{}\"", commands.replace('"', "\"\""))
        } else {
            commands
        };
        let line = format!(
            "{:.3},{},{},{},{},{},{},{},{},{},{},{},{},{}\r\n",
            self.elapsed(),
            opt(self.state.depth_cm),
            opt(self.state.vz),
            opt(self.state.az),
            opt(self.state.target_depth),
            opt(self.state.pid_output),
            opt(self.state.pid_base),
            self.state.buoyancy_pwm.map(|x| x.to_string()).unwrap_or_default(),
            bit(0x0002).map(|v| v.to_string()).unwrap_or_default(),
            bit(0x0004).map(|v| v.to_string()).unwrap_or_default(),
            bit(0x0080).map(|v| v.to_string()).unwrap_or_default(),
            bit(0x0100).map(|v| v.to_string()).unwrap_or_default(),
            commands,
            self.state.forward_active.map(|v| if v { "1" } else { "0" }).unwrap_or(""),
        );
        let Some(file) = self.control_log.as_mut() else { return };
        let _ = file.write_all(line.as_bytes());
        let _ = file.flush();
    }

    /// 滑块移动一格并生成要发的命令（向上 = 更浅）。
    fn nudge_depth(&mut self, deeper: bool) -> Action {
        let step = DEPTH_STEP_CM;
        self.depth_slider_cm = if deeper {
            (self.depth_slider_cm + step).min(DEPTH_MAX_CM)
        } else {
            self.depth_slider_cm.saturating_sub(step)
        };
        Action::Send(format!("l{}", self.depth_slider_cm))
    }

    pub fn on_key(&mut self, key: KeyEvent) -> Action {
        let ctrl = key.modifiers.contains(KeyModifiers::CONTROL);

        // ── 任何时候都生效：急停、退出、切串口、日志开关 ──
        match key.code {
            KeyCode::Char('c') | KeyCode::Char('q') if ctrl => return Action::Quit,
            KeyCode::Esc => {
                self.input = None;
                return Action::Send("s".into());
            }
            KeyCode::F(2) => {
                self.show_panel_lines = !self.show_panel_lines;
                return Action::None;
            }
            KeyCode::F(3) => return Action::Reselect,
            KeyCode::F(4) => {
                self.toggle_control_logging();
                return Action::None;
            }
            KeyCode::F(5) => {
                self.side_scroll = self.side_scroll.saturating_sub(4);
                return Action::None;
            }
            KeyCode::F(6) => {
                self.side_scroll = self.side_scroll.saturating_add(4);
                return Action::None;
            }
            KeyCode::PageUp => {
                self.input = None;
                return self.nudge_depth(false);
            }
            KeyCode::PageDown => {
                self.input = None;
                return self.nudge_depth(true);
            }
            _ => {}
        }

        // ── ":" 输入框打开时：字母进输入框 ──
        if let Some(buf) = self.input.as_mut() {
            return match key.code {
                KeyCode::Enter => {
                    let cmd = buf.trim().to_string();
                    self.input = None;
                    if cmd.is_empty() { Action::None } else { Action::Send(cmd) }
                }
                KeyCode::Backspace => {
                    buf.pop();
                    if buf.is_empty() {
                        self.input = None;   // 退完即关，回到直发模式
                    }
                    Action::None
                }
                KeyCode::Up => {
                    if let Some(c) = self.last_cmd.clone() {
                        self.input = Some(c);
                    }
                    Action::None
                }
                KeyCode::Char(c) if !ctrl => {
                    if buf.len() < 60 {
                        buf.push(c);
                    }
                    Action::None
                }
                _ => Action::None,
            };
        }

        // ── 直发模式：按什么就发什么 ──
        match key.code {
            // 激活底部"串口发送"输入框：只用回车（它本身不是任何机器人命令）。
            KeyCode::Enter => {
                self.input = Some(String::new());
                Action::None
            }
            KeyCode::Char(' ') => Action::Send("s".into()),
            KeyCode::Up => self.nudge_depth(false),
            KeyCode::Down => self.nudge_depth(true),
            KeyCode::Char(c) if !ctrl => Action::Send(c.to_lowercase().to_string()),
            _ => Action::None,
        }
    }
}

/// 펌웨어는 호환성을 위해 기존 중국어 응답도 계속 보낼 수 있다.
/// 파싱은 원문으로 수행하고, 콘솔 로그에 표시할 때만 알려진 상태 문구를 번역한다.
fn localize_rx_line(line: &str) -> String {
    let mut out = line.to_string();
    for (from, to) in [
        ("全部传感器显示", "센서 패널 표시"),
        ("传感器故障", "센서 오류"),
        ("故障", "오류"),
        ("满电", "완충"),
        ("偏低", "낮음"),
        ("低电！", "배터리 부족!"),
        ("未检测", "감지 안 됨"),
        ("未启用", "비활성"),
        ("收起", "접힘"),
        ("开", "열림"),
        ("已关闭", "꺼짐"),
        ("正常", "정상"),
    ] {
        out = out.replace(from, to);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::crossterm::event::KeyEvent;

    fn app() -> App {
        App::new("COM9".into(), 115_200, None)
    }
    fn press(a: &mut App, code: KeyCode) -> Option<String> {
        match a.on_key(KeyEvent::new(code, KeyModifiers::NONE)) {
            Action::Send(c) => Some(c),
            _ => None,
        }
    }

    #[test]
    fn keys_send_directly_without_mode_switch() {
        let mut a = app();
        for c in ['w', 'a', 'd', 'j', 'k', 'g', 'c', 'q', 'h'] {
            assert_eq!(press(&mut a, KeyCode::Char(c)).as_deref(), Some(c.to_string().as_str()));
        }
        // 大写也按小写发（机器人命令统一小写）
        assert_eq!(press(&mut a, KeyCode::Char('W')).as_deref(), Some("w"));
    }

    #[test]
    fn stop_keys() {
        let mut a = app();
        assert_eq!(press(&mut a, KeyCode::Char('s')).as_deref(), Some("s"));
        assert_eq!(press(&mut a, KeyCode::Char(' ')).as_deref(), Some("s"));
        assert_eq!(press(&mut a, KeyCode::Esc).as_deref(), Some("s"));
    }

    #[test]
    fn depth_slider_steps_5cm_and_clamps() {
        let mut a = app();
        assert_eq!(press(&mut a, KeyCode::PageDown).as_deref(), Some("l5"));
        assert_eq!(press(&mut a, KeyCode::PageDown).as_deref(), Some("l10"));
        assert_eq!(press(&mut a, KeyCode::PageUp).as_deref(), Some("l5"));
        // 0 档 = 上浮到水面，再往上不会越界
        assert_eq!(press(&mut a, KeyCode::PageUp).as_deref(), Some("l0"));
        assert_eq!(press(&mut a, KeyCode::PageUp).as_deref(), Some("l0"));
        // 上限 100cm
        for _ in 0..30 {
            press(&mut a, KeyCode::PageDown);
        }
        assert_eq!(a.depth_slider_cm, DEPTH_MAX_CM);
        assert_eq!(press(&mut a, KeyCode::PageDown).as_deref(), Some("l100"));
        // ↑↓ 与 PgUp/PgDn 等价
        assert_eq!(press(&mut a, KeyCode::Up).as_deref(), Some("l95"));
        assert_eq!(press(&mut a, KeyCode::Down).as_deref(), Some("l100"));
    }

    #[test]
    fn enter_opens_input() {
        let mut a = app();
        assert!(press(&mut a, KeyCode::Enter).is_none(), "回车不应直发");
        assert!(a.input.is_some(), "回车应打开输入框");
        assert!(press(&mut a, KeyCode::Char('m')).is_none(), "输入期间字符不应直发");
        assert_eq!(a.input.as_deref(), Some("m"));
    }

    #[test]
    fn colon_opens_input_and_enter_sends() {
        let mut a = app();
        assert!(press(&mut a, KeyCode::Enter).is_none());
        for c in "mark 下水".chars() {
            assert!(press(&mut a, KeyCode::Char(c)).is_none(), "输入期间字符不应直发");
        }
        assert_eq!(press(&mut a, KeyCode::Enter).as_deref(), Some("mark 下水"));
        // 回到直发模式
        assert_eq!(press(&mut a, KeyCode::Char('w')).as_deref(), Some("w"));
    }

    #[test]
    fn esc_during_input_is_still_emergency_stop() {
        let mut a = app();
        press(&mut a, KeyCode::Char(':'));
        press(&mut a, KeyCode::Char('m'));
        assert_eq!(press(&mut a, KeyCode::Esc).as_deref(), Some("s"));
        assert!(a.input.is_none());
    }

    #[test]
    fn slider_follows_robot_reported_target() {
        let mut a = app();
        a.on_line("Target depth set to 30.0 cm".into());
        assert_eq!(a.depth_slider_cm, 30);
    }
}
