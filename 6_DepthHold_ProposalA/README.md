# Squid Robot — Proposal A 실험용 수심 제어

## 1. 개요 / 기존 버전 보존

이 폴더는 기존 프로젝트의 소스·설정을 복사해 만든 **별도 ESP-IDF 5.4.4 프로젝트**입니다.
기존 1_ESP32_MainController/, 5_DepthLearning/의 코드·모델·빌드를 수정하지 않습니다.
생성 시 기준 소스의 PID는 10 / 0.8 / 15이며 원본 69개 파일의 SHA256을
artifacts/baseline_manifest.json에 기록했습니다. 원본이 바뀌면 평가기가 이를 알립니다.

사용자가 제공한 squid-depth-hold-proposal-a.md의 5.5장 제어기와 부록 시뮬레이터를 기반으로 합니다.
가속도 기반 PD, 침강 경향 보상, 상향·하향별 효율 변환, 저출력 펄스 제어를 사용합니다.
딥러닝 추론은 사용하지 않으며 ESP-DL 의존성과 모델 바이너리도 새 빌드에 포함하지 않습니다.
**로봇 업로드와 실제 수중 시험은 수행하지 않았습니다.**

## 2. 디렉터리 구조

~~~text
6_DepthHold_ProposalA/
├── CMakeLists.txt                  별도 프로젝트 squid_robot_proposal_a
├── sdkconfig / sdkconfig.defaults  복사된 ESP-IDF 설정
├── partitions.csv                 기존과 동일한 factory / OTA 분할
├── main/                          센서·통신·OTA 등 기존 소스의 복사본
│   ├── DepthController.h/.cpp      Proposal A 교체 제어기
│   ├── CMakeLists.txt              ESP-DL 없는 소스 등록
│   └── main.cpp                   Proposal-A EXPERIMENTAL 부팅 표시
├── simulation/
│   ├── baseline/                  생성 당시 PID 제어기 동결 사본
│   ├── common/                    KalmanFilter·Calibration 동결 사본
│   ├── stubs/                     PC용 Board·Sys·잔차 0 모델
│   ├── harness/sim_main.cpp        지연·밸브 guard를 포함한 폐루프 시뮬레이터
│   ├── harness/controller_contract.cpp  제어기 기능 검사
│   ├── build.ps1                  PC 시뮬레이터 빌드
│   ├── run_evaluation.py           11개 plant × 3개 seed 비교
│   └── bin/                       PC 시뮬레이터·검사 실행 파일
├── artifacts/
│   ├── baseline_manifest.json     원본 해시·기준 이득·구현 변경 기록
│   └── simulation_report.json     시뮬레이션 결과와 한계
└── build-proposal-a/
    └── squid_robot_proposal_a.bin  별도 ESP32-S3 펌웨어
~~~

## 3. 제어 방식과 시험용 계수

수심·속도·가속도는 아래 방향이 양수이며, 새 오차 e는 현재-목표 수심입니다.
기존 목표-현재 수심 오차와 부호가 다르지만 최종 출력은 여전히 +하강 / -상향입니다.

~~~text
e      = 현재 수심 - 목표 수심
a_des  = -KP * e - KD * 수직 속도
bias   = A_REST_REF + ALPHA * (현재 수심 - DEPTH_REF) + KI * integral(e)
effort = a_des - bias
duty   = effort / G_SINK  (effort > 0)
       = effort / G_RISE  (effort < 0)
~~~

| 계수 | 초기값 | 단위 / 역할 |
|---|---:|---|
| KP | 0.09 | 오차 1cm당 원하는 가속도 cm/s² |
| KD | 0.60 | 속도에 따른 가속도 감쇠 |
| KI | 0.01 | cm·s 누적 오차당 침강 보상 가속도 |
| I_LIMIT | 1.5 | 적분 보상 가속도 절댓값 상한 cm/s² |
| G_SINK / G_RISE | 0.75 / 4.5 | 전체 듀티의 하강 / 상향 가속도 변환 계수 |
| A_REST_REF / DEPTH_REF | 1.1 / 40cm | 기준 수심의 자연 침강 가속도 |
| ALPHA | 0.03 | 수심에 따른 침강 경향 변화 |
| DUTY_MIN | 80/255 | 최소 작동 PWM에 해당하는 듀티 |
| MOD_WINDOW_MS | 1000ms | 저출력 펄스 주기 |
| FLIP_MIN_DUTY | 0.05 | 작은 반대 방향 요청의 전환 억제 |

물리 계수는 제안서의 추정값으로 **현재 로봇의 확정된 실측값이 아닙니다**.
기존 PID 이득 10/0.8/15와 단위가 달라 숫자 크기를 직접 비교하지 마십시오.
수심·속도만 제어에 사용하고 Kalman 가속도는 기록에만 남깁니다.
적분은 포화되지 않을 때만 갱신하며 목표 변경 시 유지합니다.
최소 듀티보다 작으면 PWM 80을 1초 중 일부 시간만 켜 평균 출력을 낮춥니다.
밸브 전환 후 실제 펌프가 제한되는 200ms는 시뮬레이션 actuator에 반영되어 있습니다.

제안서와의 구현 차이:

- 자동 유지 경로에서 비정상 수심·속도·가속도 숫자도 정지 처리합니다.
- 호스트와 펌웨어가 같은 제어기 파일을 빌드하도록 Board / Calibration / Sys는
  include 경로로 선택합니다. 펌웨어에서는 실제 SDK, PC에서는 stub을 사용합니다.
- 프로젝트 바이너리와 부팅 문자열을 별도로 구분합니다.
- 로컬 PC compiler는 GCC 6.3이므로 호스트 코드를 C++14 호환 모드로 빌드합니다.

## 4. 펌웨어 빌드

설치된 ESP-IDF 5.4.4 환경을 활성화한 뒤 이 프로젝트에서 실행합니다.

~~~powershell
cd D:\Squid-Robot-Delivery-20260922\6_DepthHold_ProposalA
idf.py -B build-proposal-a build
~~~

파일은 build-proposal-a/squid_robot_proposal_a.bin입니다.
1_ESP32_MainController/build-codex/squid_robot2.bin과 혼동하지 마십시오.
부팅 문자열은 Squid-ESP32 Proposal-A EXPERIMENTAL (2026-10-08)입니다.
빌드만으로 기존 로봇 펌웨어는 바뀌지 않습니다. 자동 업로드 기능은 없습니다.

## 5. PC 시뮬레이션 / 기능 검사

g++가 PATH에 있는 환경에서:

~~~powershell
cd D:\Squid-Robot-Delivery-20260922\6_DepthHold_ProposalA
powershell -NoProfile -File simulation/build.ps1
python -B simulation/run_evaluation.py
~~~

기능 검사는 가속도 출력·보정 0·PWM 80 펄스·목표 변경 시 적분 유지·수동 방향·센서 이상·
최대 수심·기압 평형·가속도 입력 비사용을 확인합니다.
평가기의 기본 환경은 이번에 확인한 C:\MinGW\bin입니다. 다른 설치 위치에서는
run_evaluation.py의 PATH 설정을 해당 컴파일러 환경에 맞추십시오.

시뮬레이션 조건은 제안서의 8개 보정 plant와 3개 stress plant, 각각 3개 noise seed입니다.
40→50→40→30→40cm 각 90초, 목표 변경 후 초기 40초를 제외한 구간을 평가합니다.
수심 샘플은 85ms, 제어 tick은 5ms입니다. 현재 파일의 수심 필터 코드를 직접 컴파일합니다.

**비교 기준은 현재 PID 10/0.8/15의 동결 사본이지만 ESP-DL 보정은 host stub으로 0입니다.**
따라서 현재 로봇의 PID＋딥러닝 전체와 직접 비교한 결과가 아닙니다.
제안서의 이전 20/1.2/30 기준과도 다르므로 동일 수치를 기대하지 마십시오.
결과·각 plant 파라미터·각 seed 지표는 artifacts/simulation_report.json을 확인하십시오.

## 6. 로그 의미 / 호환성

외부 getter와 명령 인터페이스는 유지하지만 일부 값의 의미가 바뀝니다.

| 항목 | Proposal A 의미 |
|---|---|
| pid_output / total | 부호 있는 요구 듀티 ×100, 펄스 변조 전 |
| pid_raw | 원하는 가속도 a_des, cm/s² |
| pid_base | 침강 보상 bias, cm/s² |
| residual | 항상 0; 딥러닝 없음 |
| buoyancy_pwm | 해당 순간의 실제 PWM 명령, 0~255 |

**새 pid_base는 이전 PID 출력이 아닙니다.** 새 CSV를 5_DepthLearning의 기존 loader에
그대로 입력하면 단위와 입력 의미가 맞지 않습니다. 수심 그래프는 사용할 수 있지만
제어 출력 분석·모델 학습은 분리해서 처리해야 합니다.
PC 콘솔 코드와 기존 수집 데이터는 이 작업에서 수정하지 않았습니다.

## 7. 안전 / 실험 전에 확인할 사항

수중 시험 전 최소 작동 PWM, 상향·하향 효율, 자연 침강 경향을 별도로 측정해야 합니다.
먼저 파라미터 검증 후 안전한 수심·현장 감시 조건에서 실험하십시오.
초기 물리 계수와 시뮬레이션 결과는 실제 안정성 보장이 아닙니다.

수동 상향·하향, l0 연속 상향, s의 5초 기압 평형은 기존 경로를 유지합니다.
센서 이상 정지 또는 s가 수직 운동 정지를 보장하지는 않습니다.
현재 자동 유지 경로의 상한은 기존과 같은 100cm이며, 앞선 PC 시험의 80cm 제한이나
15cm 이하 종료 조건을 이 펌웨어에 추가한 것은 아닙니다.
표면 탈출·바닥·전진 중 동작과 밸브 guard 중 열린 밸브의 효과는 충분히 모델링되지 않았습니다.
펄스 제어는 밸브 동작 횟수를 늘릴 수 있으므로 내구성과 실제 펌프 응답도 확인해야 합니다.

원본 프로젝트를 계속 사용할 때는 기존 코드와 기존 펌웨어 경로를 그대로 사용하십시오.
이 폴더를 만들고 빌드하는 작업에는 로봇 제어 명령이나 업로드가 포함되지 않습니다.
