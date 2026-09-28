//! 仪表盘绘制（ratatui）。

use std::time::Instant;

use ratatui::layout::{Alignment, Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::symbols;
use ratatui::widgets::{Axis, Block, BorderType, Borders, Chart, Dataset, Gauge, GraphType, Paragraph, Wrap};
use ratatui::Frame;

use crate::app::{App, LogKind, DEPTH_MAX_CM as SLIDER_MAX_CM, DEPTH_STEP_CM};
use crate::parser::{Buoyancy, LineKind};

const BATT_EMPTY_V: f32 = 10.5; // 3S 锂电
const BATT_FULL_V: f32 = 12.6;

fn block(title: &str) -> Block<'_> {
    Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(Color::DarkGray))
        .title(Span::styled(format!(" {title} "), Style::default().fg(Color::Cyan).bold()))
}

fn opt(v: Option<f32>, prec: usize, unit: &str) -> String {
    match v {
        Some(x) => format!("{x:.prec$} {unit}"),
        None => "--".into(),
    }
}

fn dot(on: bool, color: Color) -> Span<'static> {
    if on {
        Span::styled("●", Style::default().fg(color).bold())
    } else {
        Span::styled("○", Style::default().fg(Color::DarkGray))
    }
}

pub fn draw(f: &mut Frame, app: &App) {
    let now = Instant::now();
    let area = f.area();
    // 窗口太小就只给一句提示：硬画会把面板挤成 0 高，没有意义。
    if area.width < 50 || area.height < 14 {
        f.render_widget(
            Paragraph::new(format!(
                "창이 너무 작습니다 ({}x{})
최소 50x14 이상으로 키워 주세요",
                area.width, area.height
            ))
            .fg(Color::Yellow),
            area,
        );
        return;
    }
    // 右侧命令/参数速查栏；窗口窄时自动隐藏。
    let side_w = if area.width >= 140 { 34 } else { 0 };
    let [body, side_area] =
        Layout::horizontal([Constraint::Min(60), Constraint::Length(side_w)]).areas(area);

    // 日志固定高度，多余的行全给主区——深度标尺越高，刻度越细（最细 5cm）。
    let log_h = if area.height >= 44 {
        10
    } else if area.height >= 34 {
        7
    } else {
        5
    };
    let [header, main, log_area, hint_area, input_area] = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(17),
        Constraint::Length(log_h),
        Constraint::Length(3),
        Constraint::Length(3),
    ])
    .areas(body);

    if side_w > 0 {
        draw_side(f, app, side_area);
    }
    draw_header(f, app, header, now);

    // 左边一整列是深度：标尺 + 紧挨着的深度曲线，两者同高不分家。
    let chart_on = body.width >= 116;
    let [depth_col, chart_col, rest] = if !chart_on {
        Layout::horizontal([Constraint::Length(22), Constraint::Length(0), Constraint::Min(40)])
            .areas(main)
    } else if body.width >= 160 {
        // 宽屏：多出来的宽度曲线和声呐按 2:3 分，声呐不再独吞
        Layout::horizontal([Constraint::Length(22), Constraint::Fill(2), Constraint::Fill(3)])
            .areas(main)
    } else {
        Layout::horizontal([Constraint::Length(22), Constraint::Length(34), Constraint::Min(40)])
            .areas(main)
    };
    draw_depth(f, app, depth_col);
    if chart_on {
        draw_depth_chart(f, app, chart_col);
    }

    // 右边：上面潜艇声呐，下面状态行。
    let [sonar_col, status] =
        Layout::vertical([Constraint::Min(12), Constraint::Length(5)]).areas(rest);
    draw_sonar(f, app, sonar_col);

    // 状态行窄的时候按比例挤，别把电池那块挤没。
    let status_cons = if status.width >= 86 {
        [Constraint::Length(30), Constraint::Min(28), Constraint::Length(28)]
    } else {
        [Constraint::Fill(4), Constraint::Fill(3), Constraint::Fill(3)]
    };
    let [motion_a, imu_a, batt_a] = Layout::horizontal(status_cons).areas(status);
    draw_motion(f, app, motion_a, now);
    draw_imu(f, app, imu_a);
    draw_battery(f, app, batt_a);

    draw_log(f, app, log_area);
    draw_hints(f, hint_area);
    draw_input(f, app, input_area);
}

fn draw_side(f: &mut Frame, app: &App, area: Rect) {
    let mut lines: Vec<Line> = Vec::new();
    for g in crate::commands::GROUPS {
        lines.push(Line::from(Span::styled(
            format!("── {} ", g.title),
            Style::default().fg(Color::Cyan).bold(),
        )));
        for it in g.items {
            lines.push(Line::from(vec![
                Span::styled(format!("{:<11}", it.key), Style::default().fg(Color::Yellow)),
                Span::styled(it.desc, Style::default().fg(Color::Gray)),
            ]));
        }
        lines.push(Line::from(""));
    }

    lines.push(Line::from(Span::styled(
        "── 액추에이터 파라미터(EEPROM) ",
        Style::default().fg(Color::Cyan).bold(),
    )));
    if app.state.params.iter().all(|p| p.is_none()) {
        lines.push(Line::from(Span::styled(
            "  :pget 읽기",
            Style::default().fg(Color::DarkGray),
        )));
    } else {
        for (i, name) in crate::commands::PARAM_NAMES.iter().enumerate() {
            let val = app.state.params.get(i).copied().flatten();
            lines.push(Line::from(vec![
                Span::styled(format!("{i:>2} "), Style::default().fg(Color::DarkGray)),
                Span::styled(format!("{name:<17}"), Style::default().fg(Color::Gray)),
                Span::styled(
                    val.map(|v| v.to_string()).unwrap_or_else(|| "--".into()),
                    Style::default().fg(Color::White).bold(),
                ),
            ]));
        }
    }

    let total = lines.len() as u16;
    let inner_h = area.height.saturating_sub(2);
    let skip = total.saturating_sub(inner_h).min(app.side_scroll);
    f.render_widget(
        Paragraph::new(lines)
            .scroll((skip, 0))
            .block(block("명령 / 파라미터 (PgUp/PgDn 수심 설정, F5/F6 스크롤)")),
        area,
    );
}

/// 按键提示条（原来的底部一行，现在固定在发送框上方）。
fn draw_hints(f: &mut Frame, area: Rect) {
    let key = |k: &'static str| Span::styled(k, Style::default().fg(Color::Cyan).bold());
    f.render_widget(
        Paragraph::new(Line::from(vec![
            key("w"), Span::raw("전진 "),
            key("a/d"), Span::raw("좌/우 "),
            key("j/k"), Span::raw("하강/상승 "),
            key("y"), Span::raw("시연 "),
            key("PgUp/PgDn"), Span::raw("수심±5cm "),
            key("q"), Span::raw("수동/자동 "),
            key("g"), Span::raw("패널 "),
            key("c"), Span::raw("영점 "),
            key("Enter"), Span::raw("명령 입력 │ "),
            Span::styled("Esc/Space/s 비상 정지", Style::default().fg(Color::Red).bold()),
            Span::raw(" │ F3 포트 변경 │ F4 데이터 저장 │ Ctrl+Q 종료"),
        ]))
        .block(block("키 즉시 전송 (누른 키를 그대로 전송)")),
        area,
    );
}

fn draw_header(f: &mut Frame, app: &App, area: Rect, now: Instant) {
    let s = &app.state;
    let (link_txt, link_color) = if let Some(e) = &app.link_error {
        (format!("시리얼 포트 연결 끊김: {e}"), Color::Red)
    } else {
        match s.last_rx {
            None => ("데이터 대기 중…".to_string(), Color::Yellow),
            Some(t) => {
                let age = now.duration_since(t).as_secs_f32();
                if age < 2.5 {
                    (format!("온라인 ({age:.1}s)"), Color::Green)
                } else if !s.panel_on {
                    // 面板收起后机器人只在事件时发数据，长时间无数据是正常的
                    (format!("유휴 {age:.0}s"), Color::Gray)
                } else if age < 6.0 {
                    (format!("지연 ({age:.1}s)"), Color::Yellow)
                } else {
                    (format!("응답 없음 {age:.0}s"), Color::Red)
                }
            }
        }
    };
    let panel_age = s.last_panel.map(|t| now.duration_since(t).as_secs_f32());
    let panel_txt = match (s.panel_on, panel_age) {
        (false, _) => "패널 접힘 (g로 열기)".to_string(),
        (true, None) => "패널: 없음".to_string(),
        (true, Some(a)) => format!("패널 {a:.1}s 전"),
    };
    let line = Line::from(vec![
        Span::styled(" 🦑 Squid 콘솔 ", Style::default().fg(Color::White).bold()),
        Span::raw(format!("│ {} @{} │ 링크 ", app.port, app.baud)),
        Span::styled("●", Style::default().fg(link_color)),
        Span::styled(format!(" {link_txt} "), Style::default().fg(link_color)),
        Span::raw(format!("│ {panel_txt} │ 전송 {} 하트비트 {} ", app.tx_count, app.hb_count)),
        Span::styled(format!("ACK {} ", s.acks), Style::default().fg(Color::Green)),
        Span::styled(format!("재시도 {} ", s.retries), Style::default().fg(Color::Yellow)),
        Span::styled(
            format!("실패 {} ", s.noacks),
            Style::default().fg(if s.noacks > 0 { Color::Red } else { Color::DarkGray }),
        ),
        // 命令往返延迟：发出 → 机器人回 [OK]。正常 100~200ms，
        // 变大说明回复排在下行数据后面，或者丢包在等 Bridge 重发。
        match app.rtt_last {
            Some(ms) => Span::styled(
                format!("│ 지연 {ms}ms"),
                Style::default().fg(if ms < 300 {
                    Color::Green
                } else if ms < 1000 {
                    Color::Yellow
                } else {
                    Color::Red
                }),
            ),
            None => Span::styled("│ 지연 --", Style::default().fg(Color::DarkGray)),
        },
    ]);
    f.render_widget(
        Paragraph::new(line).block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Rounded)
                .border_style(Style::default().fg(Color::Blue)),
        ),
        area,
    );
}

/// 深度标尺：永远是完整的 0~100cm 竖直量程，0 在最上（水面）。
/// 面板有多高就用多高——刻度线连续缩放，数字只标在整刻度上
/// （高度够是 5cm，其次 10cm、25cm），所以不会出现 7cm/格这种数。
/// ▶ = 滑块设定值（0 档绿 = 上浮到水面），● = 实测深度，重合为 ◆。
fn draw_depth(f: &mut Frame, app: &App, area: Rect) {
    let s = &app.state;
    let probe = block("수심").inner(area);
    if probe.height < 6 {
        f.render_widget(block("수심"), area);
        return;
    }
    let txt_h = 3u16.min(probe.height.saturating_sub(3));
    // 最细就是 5cm 一格（21 行）；再高也不加密，只是留白。
    let rows = ((probe.height - txt_h) as usize).min((SLIDER_MAX_CM / DEPTH_STEP_CM) as usize + 1);
    // 数字刻度间隔：能塞下就 5cm，否则 10 / 25cm
    let step: u16 = if rows >= 21 {
        DEPTH_STEP_CM
    } else if rows >= 11 {
        10
    } else {
        25
    };

    let title = format!("수심 {step}cm/칸");
    let bl = block(&title);
    let inner = bl.inner(area);
    f.render_widget(bl, area);

    let target_txt = match s.target_depth {
        Some(t) => Span::styled(format!("{t:.0} cm 수심 유지"), Style::default().fg(Color::Magenta)),
        None if app.depth_slider_cm == 0 => {
            Span::styled("0 수면으로 상승", Style::default().fg(Color::Green))
        }
        None => Span::styled("--", Style::default().fg(Color::DarkGray)),
    };
    let head = vec![
        Line::from(vec![
            Span::raw("현재 "),
            match (&s.depth_fault, s.depth_cm) {
                (Some(e), _) => {
                    Span::styled(format!("⚠{e}"), Style::default().fg(Color::Red).bold())
                }
                (None, d) => Span::styled(opt(d, 2, "cm"), Style::default().fg(Color::White).bold()),
            },
        ]),
        Line::from(vec![Span::raw("목표 "), target_txt]),
        Line::from(Span::styled(
            format!("vz {}  az {}", opt(s.vz, 1, ""), opt(s.az, 1, "")),
            Style::default().fg(Color::DarkGray),
        )),
    ];
    f.render_widget(
        Paragraph::new(head.into_iter().take(txt_h as usize).collect::<Vec<_>>()),
        Rect { height: txt_h, ..inner },
    );

    // cm → 行号（连续缩放，标记落点跟着面板高度走）
    let last = (rows - 1) as f32;
    let to_row = |cm: f32| -> usize {
        ((cm.clamp(0.0, SLIDER_MAX_CM as f32) / SLIDER_MAX_CM as f32) * last).round() as usize
    };
    let set_row = to_row(app.depth_slider_cm as f32);
    let depth_row = s.depth_cm.map(to_row);
    // 每个数字刻度落在哪一行
    let mut label_of: Vec<Option<u16>> = vec![None; rows];
    let mut cm = 0u16;
    while cm <= SLIDER_MAX_CM {
        label_of[to_row(cm as f32)] = Some(cm);
        cm += step;
    }

    let mut out: Vec<Line> = Vec::new();
    for r in 0..rows {
        let is_set = r == set_row;
        let is_depth = depth_row == Some(r);
        let marker = match (is_set, is_depth) {
            (true, true) => "◆",
            (true, false) => "▶",
            (false, true) => "●",
            _ => " ",
        };
        let mstyle = if is_set && app.depth_slider_cm == 0 {
            Style::default().fg(Color::Green).bold()
        } else if is_set {
            Style::default().fg(Color::Magenta).bold()
        } else if is_depth {
            Style::default().fg(Color::Cyan).bold()
        } else {
            Style::default()
        };
        let (num, tick, lstyle) = match label_of[r] {
            Some(v) => (
                format!("{v:>3}"),
                "┤",
                if v % 25 == 0 {
                    Style::default().fg(Color::Gray)
                } else {
                    Style::default().fg(Color::DarkGray)
                },
            ),
            None => ("   ".to_string(), "┊", Style::default().fg(Color::DarkGray)),
        };
        let tag = match (is_set, is_depth) {
            (true, true) => " 도달",
            (true, false) => " 설정",
            (false, true) => " 실측",
            _ => "",
        };
        out.push(Line::from(vec![
            Span::styled(num, lstyle),
            Span::styled(tick, lstyle),
            Span::styled(marker, mstyle),
            Span::styled(
                tag,
                if is_set {
                    Style::default().fg(Color::Magenta)
                } else {
                    Style::default().fg(Color::Cyan)
                },
            ),
        ]));
    }
    f.render_widget(
        Paragraph::new(out),
        Rect { y: inner.y + txt_h, height: inner.height - txt_h, ..inner },
    );
}

fn draw_motion(f: &mut Frame, app: &App, area: Rect, now: Instant) {
    let s = &app.state;
    let buoy = match s.buoyancy {
        Buoyancy::Stop => Span::styled("정지", Style::default().fg(Color::DarkGray)),
        Buoyancy::Ascend => Span::styled("상승(수동)", Style::default().fg(Color::Green)),
        Buoyancy::Descend => Span::styled("하강(수동)", Style::default().fg(Color::Yellow)),
        Buoyancy::Hold => Span::styled("수심 유지", Style::default().fg(Color::Magenta)),
    };
    let mode = s.control_mode.clone().unwrap_or_else(|| "MANUAL?".into());
    let mode_style = if mode == "AUTO" {
        Style::default().fg(Color::Yellow).bold()
    } else {
        Style::default().fg(Color::White)
    };
    let mut lines = vec![
        Line::from(vec![Span::raw("모드   "), Span::styled(mode, mode_style)]),
        Line::from(vec![
            Span::raw("전진   "),
            dot(s.forward_on, Color::Green),
            Span::raw(if s.forward_on { " 실행" } else { " 정지" }),
        ]),
        Line::from(vec![Span::raw("부상   "), buoy]),
        Line::from(vec![
            Span::raw("Minima "),
            dot(s.motion.fwd, Color::Green),
            Span::raw("전진 "),
            dot(s.motion.turn, Color::Cyan),
            Span::raw("회전 "),
            dot(s.motion.buoy, Color::Yellow),
            Span::raw("부상"),
        ]),
    ];
    if let Some(rem) = s.balance_remaining(now) {
        lines.push(Line::from(Span::styled(
            format!("⚠ 비상 정지 균형 조정 중 {:.1}s (잠김)", rem.as_secs_f32()),
            Style::default().fg(Color::Black).bg(Color::Red).bold(),
        )));
    }
    if let Some(e) = &s.last_error {
        lines.push(Line::from(Span::styled(e.clone(), Style::default().fg(Color::Red))));
    }
    f.render_widget(Paragraph::new(lines).block(block("이동")).wrap(Wrap { trim: true }), area);
}


/// 距离越近越危险：红 <20cm，黄 <50cm，绿 更远，灰 离线。
fn dist_style(cm: Option<f32>) -> Style {
    match cm {
        None => Style::default().fg(Color::DarkGray),
        Some(d) if d < 20.0 => Style::default().fg(Color::Red).bold(),
        Some(d) if d < 50.0 => Style::default().fg(Color::Yellow),
        Some(_) => Style::default().fg(Color::Green),
    }
}

fn dist_text(cm: Option<f32>) -> String {
    match cm {
        Some(d) => format!("{d:.0}cm"),
        None => "오프라인".into(),
    }
}

/// 声呐量程下限：三个方向至少画到 200cm。
const SONAR_MAX_CM: f32 = 200.0;

/// 把 `text` 覆盖写进以空格填充的刻度标签行。
fn place(buf: &mut [char], at: usize, text: &str) {
    for (k, ch) in text.chars().enumerate() {
        if at + k < buf.len() {
            buf[at + k] = ch;
        }
    }
}

/// 中间：机器人就是中心那个圆，前/左/右三向超声波按刻度画出去。
fn draw_sonar(f: &mut Frame, app: &App, area: Rect) {
    let s = &app.state;
    let b = block("소나 평면도 (눈금 cm)");
    let inner = b.inner(area);
    f.render_widget(b, area);
    if inner.width < 34 || inner.height < 7 {
        return;
    }

    let w = inner.width as usize;
    let mid = w / 2;
    let pre_w = 6usize; // 左侧 "{:>5} " 数值栏宽度
    let mut out: Vec<Line> = Vec::new();

    // ── 前向：顶远底近，每行左边标这一格代表的距离 ──
    let front_rows = (inner.height as usize).saturating_sub(4).clamp(1, 10);
    let need = SONAR_MAX_CM / front_rows as f32;
    let front_cell = [25.0f32, 50.0, 100.0, 200.0]
        .into_iter()
        .find(|c| *c >= need)
        .unwrap_or(200.0);
    let fstyle = dist_style(s.us[0]);
    let front_fill = match s.us[0] {
        None => 0,
        Some(d) => ((d / front_cell).round() as usize).clamp(1, front_rows),
    };
    let head = format!("전방 {}", dist_text(s.us[0]));
    out.push(Line::from(Span::styled(
        format!("{}{}", " ".repeat(mid.saturating_sub(head.chars().count() / 2)), head),
        fstyle,
    )));
    for r in 0..front_rows {
        let cm = front_cell * (front_rows - r) as f32;
        let ch = if s.us[0].is_none() {
            "┆"
        } else if r >= front_rows - front_fill {
            "┃"
        } else {
            "╷"
        };
        out.push(Line::from(vec![
            Span::styled(
                // "{:>4}┄" 占 5 列，所以往前留 5 列，竖条正好落在 mid
                format!("{}{:>4}┄", " ".repeat(mid.saturating_sub(5)), cm as u32),
                Style::default().fg(Color::DarkGray),
            ),
            Span::styled(ch, fstyle),
        ]));
    }

    // ── 中心圆 + 左右距离条（圆正好落在 mid 列，和前向竖条对齐）──
    let side_max = mid.saturating_sub(pre_w + 1).clamp(1, 40);
    // 中心圆要正好落在 mid 列（和前向竖条同一列），整组往右推。
    let pad = mid.saturating_sub(pre_w + side_max + 1);
    let side_cell = SONAR_MAX_CM / side_max as f32;
    let side_fill = |cm: Option<f32>| -> usize {
        match cm {
            None => 0,
            Some(d) => ((d / side_cell).round() as usize).clamp(1, side_max),
        }
    };
    let (lf, rf) = (side_fill(s.us[1]), side_fill(s.us[2]));
    let left_bar = if s.us[1].is_none() {
        "┄".repeat(side_max)
    } else {
        format!("{}{}", "·".repeat(side_max - lf), "━".repeat(lf))
    };
    let right_bar = if s.us[2].is_none() {
        "┄".repeat(side_max)
    } else {
        format!("{}{}", "━".repeat(rf), "·".repeat(side_max - rf))
    };
    out.push(Line::from(vec![
        Span::raw(" ".repeat(pad)),
        Span::styled(format!("{:>5} ", dist_text(s.us[1])), dist_style(s.us[1])),
        Span::styled(left_bar, dist_style(s.us[1])),
        Span::styled("◀", dist_style(s.us[1])),
        Span::styled("●", Style::default().fg(Color::White).bold()),
        Span::styled("▶", dist_style(s.us[2])),
        Span::styled(right_bar, dist_style(s.us[2])),
        Span::styled(format!(" {}", dist_text(s.us[2])), dist_style(s.us[2])),
    ]));

    // ── 左右刻度尺：中点 100cm、外端 200cm ──
    let half = side_max / 2;
    let mut ltick: Vec<char> = vec!['┼'; side_max];
    let mut rtick: Vec<char> = vec!['┼'; side_max];
    ltick[0] = '┬';
    rtick[side_max - 1] = '┬';
    if half > 0 && half < side_max {
        ltick[side_max - half] = '┬';
        rtick[half.saturating_sub(1)] = '┬';
    }
    let gap = 3; // ◀●▶
    out.push(Line::from(Span::styled(
        format!(
            "{}{}{}{}",
            " ".repeat(pad + pre_w),
            ltick.iter().collect::<String>(),
            " ".repeat(gap),
            rtick.iter().collect::<String>()
        ),
        Style::default().fg(Color::DarkGray),
    )));

    let mut llab: Vec<char> = vec![' '; side_max];
    let mut rlab: Vec<char> = vec![' '; side_max];
    place(&mut llab, 0, "200");
    place(&mut rlab, side_max.saturating_sub(3), "200");
    if half >= 2 && half < side_max {
        place(&mut llab, (side_max - half).saturating_sub(1), "100");
        place(&mut rlab, half.saturating_sub(2), "100");
    }
    out.push(Line::from(Span::styled(
        format!(
            "{}{}{}{}",
            " ".repeat(pad + pre_w),
            llab.iter().collect::<String>(),
            " ".repeat(gap),
            rlab.iter().collect::<String>()
        ),
        Style::default().fg(Color::DarkGray),
    )));
    if !s.us_seen {
        out.push(Line::from(Span::styled(
            " 패널 데이터 대기 중 (g로 열기)",
            Style::default().fg(Color::DarkGray),
        )));
    }

    f.render_widget(Paragraph::new(out), inner);
}

/// 右侧：深度随时间变化曲线（y 取负值，所以越深画得越低）。
fn draw_depth_chart(f: &mut Frame, app: &App, area: Rect) {
    let hist = &app.state.depth_hist;
    let b = block("수심 그래프");
    if hist.len() < 2 {
        f.render_widget(Paragraph::new("수심 데이터 대기 중…").fg(Color::DarkGray).block(b), area);
        return;
    }
    let data: Vec<(f64, f64)> = hist
        .iter()
        .enumerate()
        .map(|(i, d)| (i as f64, -(*d as f64)))
        .collect();
    let max_depth = hist.iter().cloned().fold(5.0f32, f32::max);
    let y_min = -((max_depth * 1.2).ceil() as f64);
    let ds = vec![Dataset::default()
        .marker(symbols::Marker::Braille)
        .graph_type(GraphType::Line)
        .style(Style::default().fg(Color::Cyan))
        .data(&data)];
    f.render_widget(
        Chart::new(ds)
            .block(b)
            .x_axis(
                Axis::default()
                    .style(Style::default().fg(Color::DarkGray))
                    .bounds([0.0, (data.len() - 1) as f64]),
            )
            .y_axis(
                Axis::default()
                    .style(Style::default().fg(Color::DarkGray))
                    .bounds([y_min, 0.0])
                    .labels(vec![
                        Span::styled(format!("{:.0}cm", -y_min), Style::default().fg(Color::DarkGray)),
                        Span::styled("0", Style::default().fg(Color::DarkGray)),
                    ]),
            ),
        area,
    );
}


fn draw_imu(f: &mut Frame, app: &App, area: Rect) {
    let s = &app.state;
    let lines = match (&s.imu, &s.imu_fault) {
        (_, Some(e)) => vec![Line::from(Span::styled(
            format!("⚠ 오류 ({e})"),
            Style::default().fg(Color::Red).bold(),
        ))],
        (Some((r, p, y)), None) => vec![
            Line::from(vec![
                Span::raw("롤 roll  "),
                Span::styled(format!("{r:>8.2}°"), Style::default().bold()),
                Span::raw("   피치 pitch "),
                Span::styled(format!("{p:>8.2}°"), Style::default().bold()),
            ]),
            Line::from(vec![
                Span::raw("요 yaw   "),
                Span::styled(format!("{y:>8.2}°"), Style::default().bold()),
            ]),
        ],
        (None, None) => vec![Line::from(Span::styled("--", Style::default().fg(Color::DarkGray)))],
    };
    f.render_widget(Paragraph::new(lines).block(block("자세 IMU")), area);
}

fn draw_battery(f: &mut Frame, app: &App, area: Rect) {
    let s = &app.state;
    let b = block("배터리");
    let inner = b.inner(area);
    f.render_widget(b, area);
    match s.battery_v {
        Some(v) if v > 5.0 => {
            let ratio = ((v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V)).clamp(0.0, 1.0);
            let color = if v >= 11.1 {
                Color::Green
            } else if v >= 10.5 {
                Color::Yellow
            } else {
                Color::Red
            };
            f.render_widget(
                Gauge::default()
                    .ratio(ratio as f64)
                    .label(format!("{v:.2} V  {}", s.battery_tag))
                    .gauge_style(Style::default().fg(color).bg(Color::Black)),
                Rect { height: inner.height.min(1), ..inner },
            );
        }
        Some(v) => f.render_widget(
            Paragraph::new(format!("{v:.2} V  배터리 미연결/감지 안 됨")).fg(Color::DarkGray),
            inner,
        ),
        None => f.render_widget(Paragraph::new("--").fg(Color::DarkGray), inner),
    }
}

fn draw_log(f: &mut Frame, app: &App, area: Rect) {
    let title = if app.show_panel_lines { "로그 (패널 원문 포함, F2 전환)" } else { "로그 (F2로 패널 원문 표시)" };
    let b = block(title);
    let inner = b.inner(area);
    let height = inner.height as usize;
    let lines: Vec<Line> = app
        .log
        .iter()
        .filter(|l| {
            app.show_panel_lines
                || !matches!(l.kind, LogKind::Rx(LineKind::Panel) | LogKind::Rx(LineKind::Fragment))
        })
        .rev()
        .take(height)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .map(|l| {
            let style = match l.kind {
                LogKind::Tx => Style::default().fg(Color::Cyan).bold(),
                LogKind::Local => Style::default().fg(Color::Magenta),
                LogKind::Rx(LineKind::Error) => Style::default().fg(Color::Red),
                LogKind::Rx(LineKind::Warn) => Style::default().fg(Color::Yellow),
                LogKind::Rx(LineKind::Bridge) => Style::default().fg(Color::DarkGray),
                LogKind::Rx(LineKind::Event) => Style::default().fg(Color::Green),
                LogKind::Rx(LineKind::Panel) => Style::default().fg(Color::Gray).add_modifier(Modifier::DIM),
                LogKind::Rx(LineKind::Other) => Style::default(),
                LogKind::Rx(LineKind::Fragment) => Style::default().fg(Color::DarkGray).add_modifier(Modifier::DIM),
            };
            Line::from(vec![
                Span::styled(format!("{:>7.1}s ", l.t), Style::default().fg(Color::DarkGray)),
                Span::styled(l.text.clone(), style),
            ])
        })
        .collect();
    f.render_widget(Paragraph::new(lines).block(b), area);
}

fn draw_input(f: &mut Frame, app: &App, area: Rect) {
    // 常驻的串口发送框：未激活时显示提示，按 ":" 或 "/" 开始输入。
    let (content, border) = match &app.input {
        Some(buf) => (
            Line::from(vec![
                Span::styled("> ", Style::default().fg(Color::Cyan).bold()),
                Span::raw(buf.clone()),
                Span::styled("█", Style::default().fg(Color::Cyan)),
            ]),
            Color::Cyan,
        ),
        None => (
            Line::from(Span::styled(
                "Enter로 입력 시작, 다시 Enter로 전송 (예: pset fwd_interval 700 · mark 하강 · mt)",
                Style::default().fg(Color::DarkGray),
            )),
            Color::DarkGray,
        ),
    };
    let mut b = Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(border))
        .title(Span::styled(" 시리얼 전송 ", Style::default().fg(Color::Cyan).bold()));
    if let Some(p) = &app.raw_log_path {
        b = b.title_bottom(
            Line::from(format!(" 원시 로그: {p} "))
                .alignment(Alignment::Right)
                .fg(Color::DarkGray),
        );
    }
    f.render_widget(Paragraph::new(content).block(b), area);
}


#[cfg(test)]
mod size_sweep {
    use super::*;
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    /// 喂一段真实回传，再把窗口尺寸从 20x4 扫到 200x70：任何一个尺寸崩了都算 bug。
    /// （曾经的坑：电池条强行 height=1，面板被挤成 0 高时直接写出缓冲区，整个程序挂掉。）
    #[test]
    fn no_panic_replaying_real_telemetry() {
        let feed = [
            "================ ALL SENSORS ================",
            "Depth: 9.13 cm | vz=-0.17 cm/s | az=-0.16 cm/s^2",
            "Ultrasonic Front: offline",
            "Ultrasonic Left : 119.8 cm",
            "Ultrasonic Right: 21.1 cm",
            "IMU: roll=-6.40 pitch=-74.52 yaw=24.96 deg",
            "Minima: motion=IDLE",
            "Battery: 11.96 V  [正常]",
            "=============================================",
            "[OK]hb",
            "[ACK] w",
            "[Retry 1/3] g",
            "[NoACK] g",
            "Target depth set to 30.0 cm",
        ];
        let mut app = App::new("COM9".into(), 115_200, None);
        app.raw_log_path = Some("squid-logs/raw-1.log".into());
        for _ in 0..20 {
            for l in feed {
                app.on_line(l.to_string());
            }
        }
        let mut bad: Vec<(u16, u16)> = Vec::new();
        for w in 20u16..=200 {
            for h in 4u16..=70 {
                let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    let mut t = Terminal::new(TestBackend::new(w, h)).unwrap();
                    t.draw(|f| draw(f, &app)).unwrap();
                }));
                if r.is_err() {
                    bad.push((w, h));
                }
            }
        }
        assert!(bad.is_empty(), "{} 个尺寸会崩，例如 {:?}", bad.len(), &bad[..bad.len().min(5)]);
    }

    #[test]
    fn no_panic_any_size() {
        for w in [40u16, 60, 80, 100, 116, 120, 140, 150, 180, 200, 240, 300, 400] {
            for h in [10u16, 14, 20, 24, 30, 40, 46, 60, 80, 120] {
                let mut app = App::new("COM9".into(), 115_200, None);
                app.state.depth_cm = Some(37.5);
                app.state.us = [Some(84.0), Some(38.0), None];
                app.state.us_seen = true;
                for i in 0..200 {
                    app.state.depth_hist.push_back((i % 40) as f32);
                }
                let mut t = Terminal::new(TestBackend::new(w, h)).unwrap();
                t.draw(|f| draw(f, &app)).unwrap_or_else(|e| panic!("{w}x{h}: {e}"));
            }
        }
    }
}

#[cfg(test)]
mod align_check {
    use super::*;
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    /// 中心圆必须和前向竖条在同一列，任何窗口尺寸都不能裂开。
    #[test]
    fn circle_aligns_with_front_bar() {
        for (w, h) in [(120u16, 30u16), (150, 46), (180, 50), (238, 63), (300, 70)] {
            let mut app = App::new("COM9".into(), 115_200, None);
            app.state.us = [Some(80.0), Some(119.8), Some(21.1)];
            app.state.us_seen = true;
            let mut t = Terminal::new(TestBackend::new(w, h)).unwrap();
            t.draw(|f| draw(f, &app)).unwrap();
            let buf = t.backend().buffer().clone();
            // 只认 "◀●▶" 那一行里的圆（表头的 ● 不算）
            let mut circle = None;
            let mut bars: Vec<u16> = Vec::new();
            for y in 0..h {
                let mut left_arrow = None;
                for x in 0..w {
                    match buf[(x, y)].symbol() {
                        "◀" => left_arrow = Some(x),
                        "●" if left_arrow == Some(x - 1) => circle = Some(x),
                        "┃" => bars.push(x),
                        _ => {}
                    }
                }
            }
            let c = circle.unwrap_or_else(|| panic!("{w}x{h}: 没画出中心圆"));
            assert!(!bars.is_empty(), "{w}x{h}: 没画出前向竖条");
            for b in bars {
                assert_eq!(b, c, "{w}x{h}: 前向竖条在 {b} 列，中心圆在 {c} 列");
            }
        }
    }
}
