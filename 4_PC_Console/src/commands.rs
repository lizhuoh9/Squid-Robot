//! 右侧命令栏的内容：按键、机器人命令、执行器时序参数。
//!
//! 参数的真值存在执行器 EEPROM 里，控制台通过机器人回传的
//! `[PARAM] <编号> <名字> = <值>` 行同步显示。

pub struct CmdEntry {
    pub key: &'static str,
    pub desc: &'static str,
}

pub struct CmdGroup {
    pub title: &'static str,
    pub items: &'static [CmdEntry],
}

const fn e(key: &'static str, desc: &'static str) -> CmdEntry {
    CmdEntry { key, desc }
}

pub const GROUPS: &[CmdGroup] = &[
    CmdGroup {
        title: "이동 (키를 누르면 즉시 전송)",
        items: &[
            e("w", "전진 켜기/끄기"),
            e("a / d", "왼쪽 회전 / 오른쪽 회전"),
            e("j / k", "하강(흡기) / 상승(배기)"),
            e("PgUp/PgDn", "수심 설정 ±5cm"),
            e("0 단계", "수면으로 상승 (l0)"),
            e("s Space Esc", "비상 정지 + 5초 균형"),
            e("q", "수동 / 자동 장애물 회피"),
            e("y", "시연 모드 켜기/끄기(자율 분사+회피)"),
        ],
    },
    CmdGroup {
        title: "센서 / 기록 (Enter 후 입력)",
        items: &[
            e("g", "센서 패널 열기/접기"),
            e("c", "현재 수심을 0으로 보정"),
            e("mark 메모", "이벤트 표시 기록"),
            e("mt / md", "테스트(SD 기록) / 디버그(WiFi)"),
            e("stat", "SD 기록 통계"),
            e("sd", "SD 마운트 진단"),
            e("ps", "시스템 상태(스택/힙/하트비트)"),
        ],
    },
    CmdGroup {
        title: "액추에이터 타이밍 파라미터(EEPROM)",
        items: &[
            e("pget", "모든 파라미터 읽기"),
            e("pset 이름 값", "파라미터 변경, 즉시 적용"),
            e("psave", "액추에이터 EEPROM에 저장"),
        ],
    },
    CmdGroup {
        title: "통신 링크 / 보호",
        items: &[
            e("fs / fs30", "리모컨 연결 끊김 보호 조회/30초 설정"),
            e("fs0", "연결 끊김 보호 끄기"),
            e("ESP025", "로봇 HC-12 채널 변경"),
            e("HC025", "Bridge 측 채널 변경"),
            e("?HC", "HC-12 파라미터 읽기(로컬)"),
        ],
    },
    CmdGroup {
        title: "콘솔",
        items: &[
            e("Enter", "시리얼 전송 입력창 열기"),
            e("F2", "로그에 패널 원문 표시"),
            e("F3", "시리얼 포트 변경"),
            e("Ctrl+Q", "종료"),
        ],
    },
];

/// 执行器参数名（顺序即编号，与固件 MotionParams.h 一致）。
pub const PARAM_NAMES: [&str; 14] = [
    "fwd_interval",
    "fwd_bal_delay",
    "fwd_bal_time",
    "fwd_bal_alt",
    "turn_duration",
    "turn_bal_delay",
    "turn_bal_time",
    "turn_bal_alt",
    "global_bal_time",
    "global_bal_alt",
    "buoy_bal_alt",
    "valve_guard",
    "pwm_period_100us",
    "link_timeout",
];
