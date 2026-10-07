//! 回传消息解析：把 HC-12 链路上的文本行映射成机器人状态。
//!
//! v2 起机器人在无线上只发一帧压缩遥测（37 字节左右），整屏面板留给它自己的
//! USB 口。帧格式见 Squid2/ESP32-IDF2/main/SensorHub.cpp。旧的 `T|...` 格式
//! 仍然认，这样同一个控制台也能连 v1 固件。
//!
//! 数据来源（都经 Minima Bridge 原样转发到 PC 串口）：
//! - 机器人 g_dbg 输出（USB + HC-12 两路）：每秒一屏 "ALL SENSORS" 面板、命令应答、
//!   `<- Minima: ...` 执行端状态、`[OK]<cmd>` 回执、`[ERR] ...`。
//! - Bridge 自己插入的：`[ACK] <cmd>`、`[Retry n/3] <cmd>`、`[NoACK] <cmd>`、信道配置结果。

use std::collections::VecDeque;
use std::time::{Duration, Instant};

const DEPTH_HIST_LEN: usize = 240;
const BALANCE_MS: u64 = 5000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Buoyancy {
    #[default]
    Stop,
    Ascend,
    Descend,
    Hold,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MotionFlags {
    pub fwd: bool,
    pub turn: bool,
    pub buoy: bool,
}

/// 一行回传的分类：决定日志区是否显示（面板行默认只进仪表盘不进日志）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineKind {
    Panel,
    Event,
    Warn,
    Error,
    Bridge,
    Other,
    /// 被 Bridge 劈开的回执碎片（默认不进日志，F2 可见）
    Fragment,
}

#[derive(Default)]
pub struct RobotState {
    // 深度
    pub depth_cm: Option<f32>,
    pub vz: Option<f32>,
    pub az: Option<f32>,
    pub depth_fault: Option<String>,
    pub depth_hist: VecDeque<f32>,
    pub target_depth: Option<f32>,
    // 超声波（None=offline）
    pub us: [Option<f32>; 3],
    pub us_seen: bool,
    // IMU
    pub imu: Option<(f32, f32, f32)>,
    pub imu_fault: Option<String>,
    // 电池
    pub battery_v: Option<f32>,
    pub battery_tag: String,
    // 运动
    pub motion: MotionFlags,
    pub forward_on: bool,
    pub buoyancy: Buoyancy,
    pub control_mode: Option<String>,
    pub estop_until: Option<Instant>,
    pub panel_on: bool,
    // 链路
    pub last_rx: Option<Instant>,
    pub last_panel: Option<Instant>,
    pub panels: u32,
    pub robot_oks: u32,
    pub acks: u32,
    pub retries: u32,
    pub noacks: u32,
    pub last_ack: Option<String>,
    pub last_noack: Option<String>,
    pub last_error: Option<String>,
    // 제어 진단 로그: 로봇이 보내는 [CTRL] 프레임에서 갱신된다.
    pub pid_output: Option<f32>,
    pub pid_raw: Option<f32>,
    pub pid_base: Option<f32>,
    pub pid_residual: Option<f32>,
    pub pid_total: Option<f32>,
    pub pid_saturated: Option<bool>,
    pub buoyancy_pwm: Option<u8>,
    pub actuator_mask: Option<u16>,
    pub forward_active: Option<bool>,
    /// 执行器 EEPROM 里的时序参数（机器人 pget 后回传，按编号存）
    pub params: [Option<u16>; 16],
    /// Bridge 在匹配到 "[OK]" 的瞬间插入换行 + "[ACK] cmd" + 换行，把机器人的 "[OK]cmd" 劈成
    /// "[OK]" / "[ACK] cmd" / "cmd" 三行。0=无，1=刚见裸 [OK]，2=又见 [ACK]，下一行是剩余部分。
    ok_split: u8,
}

impl RobotState {
    pub fn new() -> Self {
        Self { panel_on: true, ..Default::default() }
    }

    /// 急停气压平衡剩余时间（锁定期间除 s 外的命令都会被机器人拒绝）。
    pub fn balance_remaining(&self, now: Instant) -> Option<Duration> {
        self.estop_until.and_then(|t| t.checked_duration_since(now)).filter(|d| !d.is_zero())
    }

    /// 解析一行，更新状态，返回分类。
    pub fn parse_line(&mut self, raw: &str, now: Instant) -> LineKind {
        self.last_rx = Some(now);
        let line = raw.trim_end_matches(['\r', '\n']);
        let t = line.trim();
        if t.is_empty() {
            return LineKind::Panel;
        }

        // ── 제어 진단 프레임 ──────────────────────────────────────
        // [CTRL] depth_valid=1 depth=... vz=... az=... target_valid=...
        //        target=... pid=... mask_valid=1 mask=...
        if let Some(rest) = t.strip_prefix("[CTRL]") {
            self.forward_active = None;
            let mut depth_valid = false;
            let mut target_valid = false;
            let mut mask_valid = false;
            let mut target_value = None;
            let mut mask_value = None;
            for field in rest.split_whitespace() {
                let Some((key, value)) = field.split_once('=') else { continue };
                match key {
                    "depth_valid" => depth_valid = value == "1",
                    "depth" => self.depth_cm = value.parse().ok(),
                    "vz" => self.vz = value.parse().ok(),
                    "az" => self.az = value.parse().ok(),
                    "target_valid" => target_valid = value == "1",
                    "target" => target_value = value.parse().ok(),
                    "pid" => self.pid_output = value.parse().ok(),
                    "pid_raw" => self.pid_raw = value.parse().ok(),
                    "pid_base" => self.pid_base = value.parse().ok(),
                    "pid_residual" => self.pid_residual = value.parse().ok(),
                    "pid_total" => {
                        self.pid_total = value.parse().ok();
                        self.pid_output = self.pid_total;
                    }
                    "pid_saturated" => self.pid_saturated = Some(value == "1"),
                    "buoyancy_pwm" => self.buoyancy_pwm = value.parse().ok(),
                    "mask_valid" => mask_valid = value == "1",
                    "mask" => mask_value = value.parse().ok(),
                    "forward_active" => self.forward_active = match value {
                        "0" => Some(false),
                        "1" => Some(true),
                        _ => None,
                    },
                    _ => {}
                }
            }
            if !depth_valid {
                self.depth_cm = None;
                self.vz = None;
                self.az = None;
            }
            self.target_depth = if target_valid { target_value } else { None };
            self.actuator_mask = if mask_valid { mask_value } else { None };
            return LineKind::Panel;
        }

        // ── 被 Bridge 劈开的 "[OK]cmd" ─────────────────────────
        if t == "[OK]" {
            self.robot_oks += 1;
            self.ok_split = 1;
            return LineKind::Fragment;
        }
        if let Some(cmd) = t.strip_prefix("[ACK]") {
            self.acks += 1;
            // 桥 v2.0 起命令带序号（w#7），显示时去掉，日志里看着干净
            let c = cmd.trim();
            self.last_ack = Some(c.split('#').next().unwrap_or(c).to_string());
            self.ok_split = if self.ok_split == 1 { 2 } else { 0 };
            return LineKind::Bridge;
        }
        if self.ok_split == 2 {
            // "[OK]" 的剩余部分：命令回显或 " depth target=30.0cm"
            self.ok_split = 0;
            if let Some(v) = num_after(t, "depth target=") {
                self.set_target(v);
            }
            return LineKind::Fragment;
        }
        self.ok_split = 0;

        // ── 执行器时序参数回传：[PARAM] <编号> <名字> = <值> ──
        if let Some(rest) = t.strip_prefix("[PARAM]") {
            let mut it = rest.split_whitespace();
            if let (Some(id), Some(_name), Some(eq), Some(val)) =
                (it.next(), it.next(), it.next(), it.next())
            {
                if eq == "=" {
                    if let (Ok(id), Ok(val)) = (id.parse::<usize>(), val.parse::<u16>()) {
                        if id < self.params.len() {
                            self.params[id] = Some(val);
                        }
                    }
                }
            }
            return LineKind::Event;
        }

        // ── Bridge 插入的消息 ────────────────────────────────
        if t.starts_with("[Retry") {
            self.retries += 1;
            return LineKind::Warn;
        }
        if let Some(cmd) = t.strip_prefix("[NoACK]") {
            self.noacks += 1;
            self.last_noack = Some(cmd.trim().to_string());
            return LineKind::Error;
        }
        if t.starts_with("[HC-12]") || t.starts_with("[Bridge]") {
            return LineKind::Bridge;
        }

        // ── 无线遥测帧 v2（NMEA 风格）──────────────────────────
        // $T,<深度mm>,<vz>,<az>,<前cm>,<左cm>,<右cm>,<roll>,<pitch>,<yaw>,<电池cV>,<运动>*<校验>
        // 全整数按位置认，空字段=该传感器无效，*XX 是 $ 与 * 之间的异或校验。
        // 校验不过直接丢：链路串了字宁可少一帧，也别把乱数字画到仪表盘上。
        if t.starts_with("$T,") {
            return match parse_nmea_telemetry(t) {
                Some(fields) => {
                    self.apply_telemetry(&fields, now);
                    LineKind::Panel
                }
                None => LineKind::Other,   // 校验失败/字段残缺
            };
        }

        // ── 无线紧凑遥测（v1 固件的格式，保留兼容）──────────────
        // HC-12 只有 9600 波特且是半双工，整屏面板（323 字节 = 0.34s 空口）会把
        // 链路占死、命令发不进去。所以机器人无线上只发这一行（~70 字节），整屏
        // 面板留给它自己的 USB 口。格式见 ESP32-IDF/main/SensorHub.cpp。
        //   T|d=0.21|v=-0.01|a=-0.05|u=-1,-1,-1|i=-2.8,58.0,-125.3|b=11.97|m=IDLE
        if let Some(rest) = t.strip_prefix("T|") {
            self.parse_compact(rest, now);
            return LineKind::Panel;
        }

        // ── 传感器面板（USB 直连时才会看到整屏）──────────────────
        if t.contains("ALL SENSORS") {
            self.last_panel = Some(now);
            self.panels += 1;
            return LineKind::Panel;
        }
        if self.in_panel_footer(t) {
            return LineKind::Panel;
        }
        if let Some(rest) = t.strip_prefix("Depth:") {
            self.parse_depth(rest.trim());
            return LineKind::Panel;
        }
        if let Some(rest) = t.strip_prefix("Ultrasonic") {
            self.parse_ultrasonic(rest);
            return LineKind::Panel;
        }
        if let Some(rest) = t.strip_prefix("IMU:") {
            self.parse_imu(rest.trim());
            return LineKind::Panel;
        }
        if t.starts_with("IMU UART") || (t.starts_with("roll=") && t.contains("pitch=")) {
            return LineKind::Panel;
        }
        if let Some(rest) = t.strip_prefix("Minima: motion=") {
            self.motion = parse_motion(rest);
            return LineKind::Panel;
        }
        if let Some(rest) = t.strip_prefix("Battery:") {
            self.battery_v = num_after(rest, "");
            let tag = rest
                .find('[')
                .and_then(|a| rest[a + 1..].find(']').map(|b| rest[a + 1..a + 1 + b].to_string()))
                .unwrap_or_default();
            self.battery_tag = localize_battery_tag(&tag).to_string();
            return LineKind::Panel;
        }

        // ── 执行端状态变化 ────────────────────────────────────
        if let Some(rest) = t.strip_prefix("<- Minima:") {
            self.motion = parse_motion(rest);
            return LineKind::Event;
        }

        // ── 机器人回执 / 错误 ──────────────────────────────────
        if let Some(rest) = t.strip_prefix("[OK]") {
            self.robot_oks += 1;
            if let Some(v) = num_after(rest, "depth target=") {
                self.set_target(v);
            }
            return LineKind::Bridge;
        }
        if t.starts_with("[ERR]") {
            self.last_error = Some(t.to_string());
            return LineKind::Error;
        }

        // ── 命令应答（与 CommandHandler 文案一一对应）────────────
        match t {
            "Forward started." => {
                self.forward_on = true;
                return LineKind::Event;
            }
            "Ascending..." => {
                self.buoyancy = Buoyancy::Ascend;
                self.target_depth = None;
                return LineKind::Event;
            }
            "Descending..." => {
                self.buoyancy = Buoyancy::Descend;
                self.target_depth = None;
                return LineKind::Event;
            }
            "Ascend stopped." | "Descend stopped." => {
                self.buoyancy = Buoyancy::Stop;
                return LineKind::Event;
            }
            "Global balance complete. System ready." => {
                self.estop_until = None;
                return LineKind::Event;
            }
            "Depth zero recalibrated." | "Startup depth auto-calibration complete." => {
                self.depth_hist.clear();
                return LineKind::Event;
            }
            _ => {}
        }
        if t.starts_with("Forward stop") {
            self.forward_on = false;
            return LineKind::Event;
        }
        if t.starts_with("EMERGENCY STOP") {
            self.forward_on = false;
            self.buoyancy = Buoyancy::Stop;
            self.target_depth = None;
            self.control_mode = Some("MANUAL".into());
            self.estop_until = Some(now + Duration::from_millis(BALANCE_MS));
            return LineKind::Warn;
        }
        if let Some(v) = num_after(t, "Target depth set to ") {
            self.set_target(v);
            return LineKind::Event;
        }
        if let Some(rest) = t.strip_prefix("Control mode:") {
            self.control_mode = Some(rest.trim().to_string());
            return LineKind::Event;
        }
        if t.starts_with("全部传感器显示") || t.starts_with("센서 패널 표시") {
            self.panel_on = t.contains('开') || t.contains("열림") || t.contains("켜짐");
            return LineKind::Event;
        }
        if t.starts_with("Commands locked") || t.starts_with("AUTO mode is active")
            || t.starts_with("Unknown manual command")
        {
            return LineKind::Warn;
        }
        if t.starts_with("-----") {
            return LineKind::Panel;
        }
        LineKind::Other
    }

    fn in_panel_footer(&self, t: &str) -> bool {
        // 面板尾部是一整行 '='（45 个）
        t.len() >= 20 && t.chars().all(|c| c == '=')
    }

    /// 把 v2 遥测帧的整数字段写进状态。单位：深度/速度/加速度 0.1cm，
    /// 超声波 cm，角度 0.1 度，电池 0.01V，最后一位是运动位掩码。
    fn apply_telemetry(&mut self, f: &[Option<i32>], now: Instant) {
        self.last_panel = Some(now);
        self.panels += 1;

        match f[0] {
            Some(mm) => {
                let d = mm as f32 / 10.0;
                self.depth_fault = None;
                self.depth_cm = Some(d);
                if self.depth_hist.len() == DEPTH_HIST_LEN {
                    self.depth_hist.pop_front();
                }
                self.depth_hist.push_back(d);
            }
            None => {
                self.depth_cm = None;
                self.depth_fault = Some("오류".into());
            }
        }
        self.vz = f[1].map(|v| v as f32 / 10.0);
        self.az = f[2].map(|v| v as f32 / 10.0);

        self.us_seen = true;
        for i in 0..3 {
            self.us[i] = f[3 + i].map(|v| v as f32);
        }

        match (f[6], f[7], f[8]) {
            (Some(r), Some(p), Some(y)) => {
                self.imu = Some((r as f32 / 10.0, p as f32 / 10.0, y as f32 / 10.0));
                self.imu_fault = None;
            }
            _ => {
                self.imu = None;
                self.imu_fault = Some("오류".into());
            }
        }

        self.battery_v = f[9].map(|v| v as f32 / 100.0);
        self.battery_tag = match self.battery_v {
            Some(v) if v >= 12.0 => "완충",
            Some(v) if v >= 11.1 => "정상",
            Some(v) if v >= 10.5 => "낮음",
            Some(v) if v > 5.0 => "⚠ 배터리 부족!",
            _ => "감지 안 됨",
        }
        .to_string();

        let m = f[10].unwrap_or(0);
        self.motion = MotionFlags {
            fwd: m & 0x01 != 0,
            turn: m & 0x02 != 0,
            buoy: m & 0x04 != 0,
        };
    }

    /// 解析无线紧凑遥测的字段部分（不含开头的 "T|"）。
    /// 每段是 <键>=<值>；传感器无效为 "x"，超声波离线为 -1。
    fn parse_compact(&mut self, rest: &str, now: Instant) {
        self.last_panel = Some(now);
        self.panels += 1;

        for field in rest.split('|') {
            let Some((key, val)) = field.split_once('=') else { continue };
            let num = || val.trim().parse::<f32>().ok();
            match key.trim() {
                "d" => {
                    if let Some(d) = num() {
                        self.depth_fault = None;
                        self.depth_cm = Some(d);
                        if self.depth_hist.len() == DEPTH_HIST_LEN {
                            self.depth_hist.pop_front();
                        }
                        self.depth_hist.push_back(d);
                    } else {
                        self.depth_cm = None;
                        self.vz = None;
                        self.az = None;
                        self.depth_fault = Some("오류".into());
                    }
                }
                "v" => self.vz = num(),
                "a" => self.az = num(),
                "u" => {
                    self.us_seen = true;
                    for (i, part) in val.split(',').take(self.us.len()).enumerate() {
                        // 负值 = offline（机器人端统一写 -1）
                        self.us[i] = part.trim().parse::<f32>().ok().filter(|v| *v >= 0.0);
                    }
                }
                "i" => {
                    let mut it = val.split(',').map(|p| p.trim().parse::<f32>().ok());
                    match (it.next().flatten(), it.next().flatten(), it.next().flatten()) {
                        (Some(r), Some(p), Some(y)) => {
                            self.imu = Some((r, p, y));
                            self.imu_fault = None;
                        }
                        _ => {
                            self.imu = None;
                            self.imu_fault = Some("오류".into());
                        }
                    }
                }
                "b" => {
                    self.battery_v = num();
                    // 紧凑行不带 [满电]/[偏低] 这种文字标签，按同样的阈值本地补上，
                    // 阈值与 ESP32-IDF/main/SensorHub.cpp 的 renderAll() 保持一致。
                    self.battery_tag = match self.battery_v {
                        Some(v) if v >= 12.0 => "완충",
                        Some(v) if v >= 11.1 => "정상",
                        Some(v) if v >= 10.5 => "낮음",
                        Some(v) if v > 5.0 => "⚠ 배터리 부족!",
                        _ => "감지 안 됨",
                    }
                    .to_string();
                }
                "m" => self.motion = parse_motion(val),
                _ => {}
            }
        }
    }

    fn set_target(&mut self, v: f32) {
        self.target_depth = Some(v);
        self.buoyancy = Buoyancy::Hold;
    }

    fn parse_depth(&mut self, rest: &str) {
        if rest.starts_with("disabled") {
            self.depth_cm = None;
            self.depth_fault = Some("비활성".into());
            return;
        }
        if rest.contains("故障") || rest.contains("오류") || rest.starts_with('⚠') {
            self.depth_cm = None;
            self.vz = None;
            self.az = None;
            self.depth_fault = Some(paren(rest).unwrap_or("오류").to_string());
            return;
        }
        self.depth_fault = None;
        self.depth_cm = num_after(rest, "");
        self.vz = num_after(rest, "vz=");
        self.az = num_after(rest, "az=");
        if let Some(d) = self.depth_cm {
            if self.depth_hist.len() == DEPTH_HIST_LEN {
                self.depth_hist.pop_front();
            }
            self.depth_hist.push_back(d);
        }
    }

    fn parse_ultrasonic(&mut self, rest: &str) {
        // " Front: 12.3 cm" / " Left : offline" / " Right: ..."
        let Some((name, val)) = rest.split_once(':') else { return };
        let idx = match name.trim() {
            "Front" => 0,
            "Left" => 1,
            "Right" => 2,
            _ => return,
        };
        self.us_seen = true;
        self.us[idx] = if val.contains("offline") { None } else { num_after(val, "") };
    }

    fn parse_imu(&mut self, rest: &str) {
        if rest.contains("故障") || rest.contains("오류") || rest.starts_with('⚠') {
            self.imu = None;
            self.imu_fault = Some(paren(rest).unwrap_or("오류").to_string());
            return;
        }
        match (num_after(rest, "roll="), num_after(rest, "pitch="), num_after(rest, "yaw=")) {
            (Some(r), Some(p), Some(y)) => {
                self.imu = Some((r, p, y));
                self.imu_fault = None;
            }
            _ => {}
        }
    }
}

fn parse_motion(s: &str) -> MotionFlags {
    MotionFlags { fwd: s.contains("FWD"), turn: s.contains("TURN"), buoy: s.contains("BUOY") }
}

/// 括号里的内容："⚠ 传感器故障 (stale)" -> "stale"
fn paren(s: &str) -> Option<&str> {
    let a = s.find('(')?;
    let b = s[a + 1..].find(')')?;
    Some(&s[a + 1..a + 1 + b])
}

/// 在 key 之后找第一个数字（key 为空则从头找）。
/// 校验并拆开一帧 v2 遥测。返回 11 个字段（空字段为 None）；
/// 校验和不匹配、字段数不对、或有字段既非空又不是整数，一律返回 None 丢弃整帧。
fn parse_nmea_telemetry(line: &str) -> Option<Vec<Option<i32>>> {
    let body = line.strip_prefix('$')?;
    let (payload, checksum) = body.split_once('*')?;
    let want = u8::from_str_radix(checksum.trim(), 16).ok()?;
    let got = payload.bytes().fold(0u8, |acc, b| acc ^ b);
    if got != want {
        return None;
    }

    let mut parts = payload.split(',');
    if parts.next()? != "T" {
        return None;
    }
    let mut out = Vec::with_capacity(11);
    for part in parts {
        let p = part.trim();
        if p.is_empty() {
            out.push(None);
        } else {
            out.push(Some(p.parse::<i32>().ok()?));
        }
    }
    if out.len() != 11 {
        return None;
    }
    Some(out)
}

pub fn num_after(s: &str, key: &str) -> Option<f32> {
    let start = if key.is_empty() { 0 } else { s.find(key)? + key.len() };
    let tail = &s[start..];
    let begin = tail.find(|c: char| c.is_ascii_digit() || c == '-' || c == '.')?;
    let num: String = tail[begin..]
        .chars()
        .enumerate()
        .take_while(|(i, c)| c.is_ascii_digit() || *c == '.' || (*i == 0 && *c == '-'))
        .map(|(_, c)| c)
        .collect();
    num.parse().ok()
}

fn localize_battery_tag(tag: &str) -> &str {
    match tag.trim() {
        "满电" => "완충",
        "正常" => "정상",
        "偏低" => "낮음",
        "低电！" | "⚠ 低电！" => "⚠ 배터리 부족!",
        "未检测" => "감지 안 됨",
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn st() -> (RobotState, Instant) {
        (RobotState::new(), Instant::now())
    }

    #[test]
    fn panel_real_lines() {
        let (mut s, now) = st();
        // 以下为 2026-09-19 真机 HC-12 链路上抓到的原文
        assert_eq!(s.parse_line("================ ALL SENSORS ================\r", now), LineKind::Panel);
        s.parse_line("Depth: 0.16 cm | vz=0.05 cm/s | az=-0.04 cm/s^2", now);
        s.parse_line("Ultrasonic Front: offline", now);
        s.parse_line("Ultrasonic Left : 123.4 cm", now);
        s.parse_line("Ultrasonic Right: offline", now);
        s.parse_line("IMU: roll=-172.45 pitch=87.94 yaw=-175.66 deg", now);
        s.parse_line("Minima: motion=FWD ", now);
        s.parse_line("Battery: 11.95 V  [正常]", now);
        assert_eq!(s.parse_line("=============================================", now), LineKind::Panel);
        assert_eq!(s.depth_cm, Some(0.16));
        assert_eq!(s.vz, Some(0.05));
        assert_eq!(s.az, Some(-0.04));
        assert_eq!(s.us, [None, Some(123.4), None]);
        assert_eq!(s.imu, Some((-172.45, 87.94, -175.66)));
        assert!(s.motion.fwd && !s.motion.turn);
        assert_eq!(s.battery_v, Some(11.95));
        assert_eq!(s.battery_tag, "정상");
        assert_eq!(s.panels, 1);
        assert_eq!(s.depth_hist.len(), 1);
    }

    #[test]
    fn compact_telemetry_line() {
        let (mut s, now) = st();
        // 无线上机器人只发这一行（ESP32-IDF/main/SensorHub.cpp renderCompactHc12）
        let line = "T|d=0.21|v=-0.01|a=-0.05|u=-1,123.4,-1|i=-2.8,58.0,-125.3|b=11.97|m=FWD+BUOY";
        assert_eq!(s.parse_line(line, now), LineKind::Panel);
        assert_eq!(s.depth_cm, Some(0.21));
        assert_eq!(s.vz, Some(-0.01));
        assert_eq!(s.az, Some(-0.05));
        assert_eq!(s.us, [None, Some(123.4), None]);
        assert_eq!(s.imu, Some((-2.8, 58.0, -125.3)));
        assert_eq!(s.battery_v, Some(11.97));
        assert_eq!(s.battery_tag, "정상");
        assert!(s.motion.fwd && s.motion.buoy && !s.motion.turn);
        assert_eq!(s.panels, 1);
        assert_eq!(s.depth_hist.len(), 1);
        assert!(s.last_panel.is_some());

        // 传感器无效时机器人写 x
        s.parse_line("T|d=x|v=x|a=x|u=-1,-1,-1|i=x|b=11.90|m=IDLE", now);
        assert_eq!(s.depth_cm, None);
        assert!(s.depth_fault.is_some());
        assert_eq!(s.imu, None);
        assert!(s.imu_fault.is_some());
        assert_eq!(s.motion, MotionFlags::default());
    }

    #[test]
    fn nmea_telemetry_frame() {
        let (mut s, now) = st();
        // 深度 0.2cm，三个超声波全离线，IMU 有数，电池 12.04V，IDLE
        let line = "$T,2,1,0,,,,-57,-136,-347,1204,0*57";
        assert_eq!(s.parse_line(line, now), LineKind::Panel);
        assert_eq!(s.depth_cm, Some(0.2));
        assert_eq!(s.vz, Some(0.1));
        assert_eq!(s.us, [None, None, None]);
        assert_eq!(s.imu, Some((-5.7, -13.6, -34.7)));
        assert_eq!(s.battery_v, Some(12.04));
        assert_eq!(s.battery_tag, "완충");
        assert_eq!(s.motion, MotionFlags::default());
        assert_eq!(s.panels, 1);

        // 深度传感器故障（空字段），三个超声波有数，前进+浮沉同时在跑
        let line = "$T,,,,45,120,88,-12,3,1795,1198,5*69";
        assert_eq!(s.parse_line(line, now), LineKind::Panel);
        assert_eq!(s.depth_cm, None);
        assert!(s.depth_fault.is_some());
        assert_eq!(s.us, [Some(45.0), Some(120.0), Some(88.0)]);
        assert_eq!(s.battery_tag, "정상");
        assert!(s.motion.fwd && s.motion.buoy && !s.motion.turn);
        assert_eq!(s.panels, 2);
    }

    #[test]
    fn nmea_bad_checksum_is_dropped() {
        let (mut s, now) = st();
        // 校验和对不上 = 链路串字，整帧丢掉，绝不能把乱数字画到仪表盘上
        let bad = "$T,2,1,0,,,,-57,-136,-347,1204,0*FF";
        assert_eq!(s.parse_line(bad, now), LineKind::Other);
        assert_eq!(s.depth_cm, None);
        assert_eq!(s.panels, 0);

        // 字段数不对也丢
        let short = "$T,1,2,3*0C";
        assert_eq!(s.parse_line(short, now), LineKind::Other);
        assert_eq!(s.panels, 0);
    }

    #[test]
    fn ack_strips_sequence_number() {
        let (mut s, now) = st();
        // 桥 v2.0 给命令编号，显示时去掉
        assert_eq!(s.parse_line("[ACK] w#7", now), LineKind::Bridge);
        assert_eq!(s.last_ack.as_deref(), Some("w"));
    }

    #[test]
    fn faults() {
        let (mut s, now) = st();
        s.parse_line("Depth: ⚠ 传感器故障 (stale)", now);
        assert_eq!(s.depth_cm, None);
        assert_eq!(s.depth_fault.as_deref(), Some("stale"));
        s.parse_line("IMU: ⚠ 传感器故障 (no data)", now);
        assert_eq!(s.imu_fault.as_deref(), Some("no data"));
        s.parse_line("Battery: 3.62 V  [未检测]", now);
        assert_eq!(s.battery_v, Some(3.62));
    }

    #[test]
    fn command_flow() {
        let (mut s, now) = st();
        s.parse_line("Forward started.", now);
        assert!(s.forward_on);
        assert_eq!(s.parse_line("[OK]w", now), LineKind::Bridge);
        assert_eq!(s.parse_line("[ACK] w", now), LineKind::Bridge);
        assert_eq!((s.robot_oks, s.acks), (1, 1));
        s.parse_line("Forward stop: pressure balance in progress...", now);
        assert!(!s.forward_on);

        s.parse_line("Target depth set to 30.0 cm", now);
        assert_eq!(s.target_depth, Some(30.0));
        assert_eq!(s.buoyancy, Buoyancy::Hold);
        s.parse_line("Ascending...", now);
        assert_eq!(s.buoyancy, Buoyancy::Ascend);
        assert_eq!(s.target_depth, None);

        s.parse_line("[OK] depth target=25.5cm", now);
        assert_eq!(s.target_depth, Some(25.5));

        assert_eq!(
            s.parse_line("EMERGENCY STOP: global pressure balance started (5s), commands locked.", now),
            LineKind::Warn
        );
        assert!(s.balance_remaining(now).is_some());
        s.parse_line("Global balance complete. System ready.", now);
        assert!(s.balance_remaining(now).is_none());

        s.parse_line("Control mode: AUTO", now);
        assert_eq!(s.control_mode.as_deref(), Some("AUTO"));
        s.parse_line("<- Minima: TURN BUOY ", now);
        assert!(s.motion.turn && s.motion.buoy && !s.motion.fwd);
        s.parse_line("全部传感器显示: 收起", now);
        assert!(!s.panel_on);
    }

    #[test]
    fn control_forward_intent_is_explicit_and_not_stale() {
        let (mut s, now) = st();
        s.parse_line("[CTRL] depth_valid=1 depth=40.0 vz=0 az=0 target_valid=1 target=40 pid=80 pid_base=80 buoyancy_pwm=217 mask_valid=1 mask=0 forward_active=1", now);
        assert_eq!(s.forward_active, Some(true));
        s.parse_line("[CTRL] depth_valid=1 depth=40.0 vz=0 az=0 target_valid=1 target=40 pid=80 pid_base=80 buoyancy_pwm=217 mask_valid=1 mask=6 forward_active=0", now);
        assert_eq!(s.forward_active, Some(false));
        s.parse_line("[CTRL] depth_valid=1 depth=40.0 target_valid=1 target=40", now);
        assert_eq!(s.forward_active, None);
    }

    #[test]
    fn split_ok_by_bridge() {
        // 2026-09-19 真机：机器人 "[OK]l30" 被 Bridge 插入 "[ACK] l30" 劈开
        let (mut s, now) = st();
        assert_eq!(s.parse_line("[OK]", now), LineKind::Fragment);
        assert_eq!(s.parse_line("[ACK] l30", now), LineKind::Bridge);
        assert_eq!(s.parse_line(" depth target=30.0cm", now), LineKind::Fragment);
        assert_eq!(s.parse_line("[OK]l30", now), LineKind::Bridge);
        assert_eq!(s.target_depth, Some(30.0));
        assert_eq!((s.robot_oks, s.acks), (2, 1));
        // 普通命令
        s.parse_line("[OK]", now);
        s.parse_line("[ACK] j", now);
        assert_eq!(s.parse_line("j", now), LineKind::Fragment);
        assert_eq!(s.parse_line("<- Minima: IDLE", now), LineKind::Event);
    }

    #[test]
    fn bridge_retry_and_fail() {
        let (mut s, now) = st();
        assert_eq!(s.parse_line("[Retry 1/3] w", now), LineKind::Warn);
        assert_eq!(s.parse_line("[NoACK] w", now), LineKind::Error);
        assert_eq!((s.retries, s.noacks), (1, 1));
        assert_eq!(s.last_noack.as_deref(), Some("w"));
        assert_eq!(s.parse_line("[ERR] Depth sensor offline, cannot set target.", now), LineKind::Error);
    }

    #[test]
    fn num_after_works() {
        assert_eq!(num_after(" 11.95 V", ""), Some(11.95));
        assert_eq!(num_after("vz=-0.00 cm/s", "vz="), Some(-0.0));
        assert_eq!(num_after("x", "vz="), None);
    }
}
