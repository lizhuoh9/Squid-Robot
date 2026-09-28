//! 串口选择界面：自动发现 Bridge，找不到或有多个时让用户选。

use std::io;
use std::time::Duration;

use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind};
use ratatui::layout::{Constraint, Layout};
use ratatui::style::{Color, Style, Stylize};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, BorderType, Borders, List, ListItem, ListState, Paragraph};
use ratatui::{DefaultTerminal, Frame};

use crate::link::{self, PortCandidate};

pub enum Pick {
    Port(String),
    Quit,
}

/// 扫描 + 选择。只有一个 Bridge 且 auto=true 时直接返回，不打扰用户。
pub fn pick(terminal: &mut DefaultTerminal, auto: bool) -> io::Result<Pick> {
    let mut ports = scan(terminal)?;
    if auto {
        let bridges: Vec<&PortCandidate> = ports.iter().filter(|c| c.is_bridge).collect();
        if bridges.len() == 1 {
            return Ok(Pick::Port(bridges[0].name.clone()));
        }
    }
    let mut state = ListState::default();
    state.select(Some(0));
    loop {
        terminal.draw(|f| draw(f, &ports, &mut state))?;
        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(k) = event::read()? {
                if k.kind != KeyEventKind::Press {
                    continue;
                }
                match k.code {
                    KeyCode::Char('q') | KeyCode::Esc => return Ok(Pick::Quit),
                    KeyCode::Char('r') => {
                        ports = scan(terminal)?;
                        state.select(Some(0));
                    }
                    KeyCode::Up => {
                        let i = state.selected().unwrap_or(0);
                        state.select(Some(i.saturating_sub(1)));
                    }
                    KeyCode::Down => {
                        let i = state.selected().unwrap_or(0);
                        state.select(Some((i + 1).min(ports.len().saturating_sub(1))));
                    }
                    KeyCode::Enter => {
                        if let Some(c) = state.selected().and_then(|i| ports.get(i)) {
                            return Ok(Pick::Port(c.name.clone()));
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

fn scan(terminal: &mut DefaultTerminal) -> io::Result<Vec<PortCandidate>> {
    terminal.draw(|f| {
        let area = f.area();
        f.render_widget(
            Paragraph::new("시리얼 포트를 검색하여 Bridge를 찾는 중입니다 (?ID 식별 프레임 전송, 무선 대역폭 사용 안 함)…")
                .block(frame_block(" 시리얼 포트 검색 "))
                .fg(Color::Cyan),
            area,
        );
    })?;
    Ok(link::discover(true))
}

fn frame_block(title: &str) -> Block<'_> {
    Block::default()
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(Color::Blue))
        .title(Span::styled(title.to_string(), Style::default().fg(Color::Cyan).bold()))
}

fn draw(f: &mut Frame, ports: &[PortCandidate], state: &mut ListState) {
    let [main, help] =
        Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).areas(f.area());

    let items: Vec<ListItem> = if ports.is_empty() {
        vec![ListItem::new(Line::from(Span::styled(
            "시리얼 포트를 찾지 못했습니다 — Bridge 연결을 확인하세요",
            Style::default().fg(Color::Red),
        )))]
    } else {
        ports
            .iter()
            .map(|c| {
                let style = if c.is_bridge {
                    Style::default().fg(Color::Green).bold()
                } else {
                    Style::default().fg(Color::Gray)
                };
                ListItem::new(Line::from(vec![
                    Span::styled(format!("{:<6}", c.name), style),
                    Span::styled(c.label.clone(), Style::default().fg(Color::DarkGray)),
                ]))
            })
            .collect()
    };

    f.render_stateful_widget(
        List::new(items)
            .block(frame_block(" 시리얼 포트 선택 (녹색은 자동 인식된 Bridge) "))
            .highlight_style(Style::default().bg(Color::Blue).fg(Color::White).bold())
            .highlight_symbol("▶ "),
        main,
        state,
    );
    f.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled("↑↓", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 선택  "),
            Span::styled("Enter", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 연결  "),
            Span::styled("r", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 다시 검색  "),
            Span::styled("q/Esc", Style::default().fg(Color::Cyan).bold()),
            Span::raw(" 종료"),
        ]))
        .block(frame_block(" 조작 ")),
        help,
    );
}
