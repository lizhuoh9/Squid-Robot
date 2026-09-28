//! 串口链路：打开 Bridge 串口，后台线程按行读取；主线程写命令。

use std::fs::File;
use std::io::{self, Read, Write};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use std::time::{Duration, Instant};

use serialport::SerialPort;

pub enum RxMsg {
    Line(String),
    Closed(String),
}

pub struct Link {
    writer: Box<dyn SerialPort>,
    pub rx: Receiver<RxMsg>,
}

impl Link {
    pub fn open(port: &str, baud: u32, raw_log: Option<File>) -> io::Result<Self> {
        let mut sp = serialport::new(port, baud)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, format!("{port} 열기 실패: {e}")))?;
        // UNO R4 的 USB 串口在 setup() 里 while(!Serial) 等 DTR，拉高才开始转发。
        let _ = sp.write_data_terminal_ready(true);
        let reader = sp
            .try_clone()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, format!("시리얼 포트 복제 실패: {e}")))?;
        let (tx, rx) = mpsc::channel();
        spawn_reader(reader, tx, raw_log);
        Ok(Self { writer: sp, rx })
    }

    /// 发送一条命令（Bridge 以 \r 或 \n 作为命令结束）。
    pub fn send(&mut self, cmd: &str) -> io::Result<()> {
        self.writer.write_all(cmd.as_bytes())?;
        self.writer.write_all(b"\n")?;
        self.writer.flush()
    }
}

fn spawn_reader(mut port: Box<dyn SerialPort>, tx: Sender<RxMsg>, mut raw_log: Option<File>) {
    thread::spawn(move || {
        let mut buf = [0u8; 1024];
        let mut line: Vec<u8> = Vec::with_capacity(256);
        loop {
            match port.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => {
                    if let Some(f) = raw_log.as_mut() {
                        let _ = f.write_all(&buf[..n]);
                    }
                    for &b in &buf[..n] {
                        if b == b'\n' {
                            let s = String::from_utf8_lossy(&line).trim_end_matches('\r').to_string();
                            line.clear();
                            if tx.send(RxMsg::Line(s)).is_err() {
                                return;
                            }
                        } else {
                            line.push(b);
                            if line.len() > 1024 {
                                // 无换行的超长噪声：强制切一行
                                let s = String::from_utf8_lossy(&line).to_string();
                                line.clear();
                                let _ = tx.send(RxMsg::Line(s));
                            }
                        }
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => {}
                Err(e) => {
                    let _ = tx.send(RxMsg::Closed(e.to_string()));
                    return;
                }
            }
        }
    });
}

// ── 自动发现 ─────────────────────────────────────────────────────────
// Bridge 固件收到本地查询 "?ID" 会回 BRIDGE_TAG 这一行（只走 USB，不经 HC-12，
// 不占无线带宽）；开机横幅里也有同样一行。据此在一堆串口里认出桥。
const BRIDGE_TAG: &str = "[BRIDGE]SQUID-HC12-BRIDGE";
const ARDUINO_VID: u16 = 0x2341;

pub struct PortCandidate {
    pub name: String,
    pub label: String,
    pub is_bridge: bool,
}

/// 枚举串口；probe=true 时逐个发 "?ID" 探测哪个是 Bridge。
pub fn discover(probe: bool) -> Vec<PortCandidate> {
    let ports = serialport::available_ports().unwrap_or_default();
    let mut out = Vec::new();
    for p in ports {
        let (label, arduino) = match &p.port_type {
            serialport::SerialPortType::UsbPort(u) => (
                format!(
                    "USB {:04x}:{:04x} {}",
                    u.vid,
                    u.pid,
                    u.product.clone().unwrap_or_default()
                ),
                u.vid == ARDUINO_VID,
            ),
            serialport::SerialPortType::BluetoothPort => ("블루투스".into(), false),
            _ => ("(검색 안 함)".into(), false),
        };
        // 只探测 Arduino 厂商号的口：Bridge 一定是 UNO R4。其它口（机器人的 CH343、
        // RGB 的 ESP32、蓝牙虚拟口）一律不打开——避免和烧录/日志抢占同一个串口。
        let is_bridge = if probe && arduino { probe_bridge(&p.port_name) } else { false };
        out.push(PortCandidate {
            name: p.port_name,
            label: if is_bridge { format!("{label}  ← Bridge") } else { label },
            is_bridge,
        });
    }
    // Bridge 排前面
    out.sort_by_key(|c| !c.is_bridge);
    out
}

/// 向 Arduino 口发 "?ID" 并等识别帧。UNO R4 是 USB CDC，要拉 DTR 才开始转发，
/// 且会因此复位，所以等久一点、顺便接开机横幅里的识别帧。
fn probe_bridge(name: &str) -> bool {
    let Ok(mut sp) = serialport::new(name, 115_200).timeout(Duration::from_millis(60)).open() else {
        return false;
    };
    let _ = sp.write_data_terminal_ready(true);
    let _ = sp.write_request_to_send(false);
    let _ = sp.write_all(b"?ID\n");
    let _ = sp.flush();

    let deadline = Instant::now() + Duration::from_millis(2500);
    let mut buf = [0u8; 256];
    let mut seen = String::new();
    while Instant::now() < deadline {
        match sp.read(&mut buf) {
            Ok(n) if n > 0 => {
                seen.push_str(&String::from_utf8_lossy(&buf[..n]));
                if seen.contains(BRIDGE_TAG) {
                    return true;
                }
                if seen.len() > 8192 {
                    seen.clear();
                }
            }
            Ok(_) => {}
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                // 复位后要等一会儿才出横幅；中途再问一次。
                let _ = sp.write_all(b"?ID\n");
            }
            Err(_) => return false,
        }
    }
    false
}
