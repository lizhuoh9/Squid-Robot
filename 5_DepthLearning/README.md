# Squid Robot — 수심 보정 딥러닝

## 1. 개요

5_DepthLearning/은 PC에서 수심 보정 모델을 학습하고 ESP32-S3용 ESP-DL로 변환하는 폴더입니다.
전진·회전 제어 모델은 포함하지 않습니다. SD 카드 없이 PC에서 수집한 CSV를 사용합니다.

현재 제어기는 artifacts/depth_model_based_v1/의 예측 모델 기반 보정을 사용합니다.
이전 PD 규칙 모방 모델은 비교·복구용으로 보존되어 있으며 현재 기본 선택은 아닙니다.
연결 설정의 기준은 ../1_ESP32_MainController/main/CMakeLists.txt입니다.

| 구성 | 실행 위치 | 역할 |
|---|---|---|
| 동역학 신경망 3개 | PC | 제어 출력에 따른 다음 수심·속도 변화를 예측 |
| 3초 후보 탐색 | PC | 목표 오차·속도·모델 불일치를 평가해 보정 정답 생성 |
| 작은 보정 신경망 | PC에서 학습, ESP32-S3에서 추론 | 5프레임 입력으로 보정 출력 계산 |
| 기본 PID | ESP32-S3 | 새 수심 측정마다 기본 제어 수행 |

현재 소스의 기본 PID는 **Kp=10 / Ki=1.2 / Kd=15**이며 상황별 배율은 유지됩니다.
Ki는 이전에 제안한 0.6이 아니라 1.2입니다. 2026-10-07 대화에서 확인한 업로드·순차
시험은 20/1.2/30 버전이며, 이후 10/1.2/15 소스의 업로드 여부는 이번 문서 작업에서
확인하지 않았습니다. 현재 v1 모델은 PID 이득을 낮추기 전에 학습한 모델입니다.

~~~text
u_base   = clamp(PID 및 속도 제한 결과, -80, 80)
residual = clamp(ESP-DL 보정 예측, -20, 20)
u_total  = clamp(u_base + residual, -100, 100)
~~~

보정 단위는 cm가 아닌 제어 출력 포인트입니다. PID와 반대 부호도 가능합니다.
수동 상향·하향은 보정 없이 ±100 출력과 PWM 255를 사용합니다.
추가 누적 오차 학습은 제거했지만 기본 PID의 I항은 사용합니다.

## 2. 디렉터리 구조

~~~text
5_DepthLearning/
├── README.md                      전체 구조·사용 방법
├── README_model_based.md          새 방식 상세 설명·검증·실험 기록
├── requirements.txt               기본 PyTorch·ONNX 의존성
├── learning/
│   ├── model.py                   공용 작은 MLP
│   ├── data.py                    이전 PD 방식 CSV·5프레임 입력 준비
│   ├── train.py                   이전 PD 교사 규칙 모방 학습
│   ├── export_espdl.py            이전 PD 모델 ESP-DL 변환
│   ├── export.py                  과거 ONNX·C++ 가중치 헤더 변환
│   └── model_based/
│       ├── data.py                연속 구간 분리·인과적 입력·동역학 표본
│       ├── train.py               동역학 학습·보정 탐색·작은 모델 학습
│       ├── export.py              새 모델 ESP-DL 변환·양자화 검증
│       └── verify.py              원본 해시·입력·구간 분리·출력 범위 검증
└── artifacts/
    ├── depth_model_based_v1/      현재 연결된 예측 기반 결과
    ├── depth_model_pd_restore_1790936043/  보존된 PD ESP-DL·펌웨어
    ├── depth_model_1790936043/    이전 PD 학습 원본·checkpoint
    ├── depth_model/              초기 학습 결과
    └── depth_*.png               수심 기록 그래프
~~~

## 3. 현재 결과 파일의 역할

artifacts/depth_model_based_v1/의 주요 파일:

| 파일 | 역할 |
|---|---|
| dynamics_model.pt | PC용 동역학 앙상블 checkpoint |
| teacher_labels.csv | 원본 위치·학습/검증 구분·탐색한 정답 보정 |
| depth_model.pt | 작은 보정 MLP의 PyTorch checkpoint |
| depth_manifest.json | 입력 순서·정규화·구조·원본 해시·가드 설정 |
| policy_windows.pt | 학습/검증/시험 입력과 교사 보정; 전용 exporter에 사용 |
| depth_model.onnx | INT8 변환 전 중간 모델 |
| depth_model_espdl.espdl | 펌웨어에 포함되는 ESP32-S3 INT8 모델 |
| DepthResidualModelConfig.h | 펌웨어 정규화·샘플링·출력 scale·가드 설정 |
| training_report.json | 예측·보정 모방 성능과 제한 사항 |
| depth_model_espdl.validation.json | float/INT8 비교·±20 표현 범위 검사 |
| depth_model_espdl.json / .info | ESP-PPQ 양자화 설정·모델 정보 |

로봇에는 .pt나 CSV를 복사하지 않습니다. **.espdl과 설정 헤더**를 빌드에 포함합니다.
두 파일은 같은 변환 결과끼리 짝을 맞춰야 합니다.
과거 export.py가 생성하는 DepthResidualModelWeights.h는 현재 ESP-DL 경로가 아닙니다.

## 4. 입력 데이터

현재 방식의 필수 CSV 열:

~~~text
elapsed_s,depth_cm,depth_speed_cm_s,depth_accel_cm_s2,target_depth_cm,
pid_base,pid_output,buoyancy_pwm,forward_active
~~~

시간은 PC의 단조 증가 시간 elapsed_s를 사용합니다.
robot_timestamp_ms와 depth_sample_ms는 필요하지 않습니다.
목표 변경, 잘못된 숫자, 수심 범위 이탈, 0.40~0.60초 밖의 간격에서 구간을 끊습니다.
현재 loader는 전진 중인 기록과 기본 출력 ±80을 넘는 수동 제어를 제외합니다.
마지막 l0 상향 구간은 그래프에는 포함하지만 자동 수심 보정 학습에서는 제외합니다.

제어 모델은 다음 순서로 각 프레임의 입력 5개를, 오래된 프레임부터 5개 연결합니다.

~~~text
depth_err_cm, depth_speed_cm_s, depth_accel_cm_s2, u_base, base_pwm_commanded
~~~

오차는 목표-현재 수심입니다. 마지막 값은 로그의 최종 PWM이 아니라 현재 기본 출력에서
계산한 PWM입니다. CSV와 추론의 시점 차이를 피하도록 Python과 C++의 계산·반올림이 같습니다.
총 입력 25개, 은닉층 24/12개, 출력 1개의 MLP입니다.

## 5. 학습 / 변환 / 검증

Python 3.10 환경에서 학습·변환했습니다. 학습에는 PyTorch·NumPy,
변환에는 ONNX·ESP-PPQ도 필요합니다. requirements.txt에는 NumPy·ESP-PPQ가 직접 명시되어 있지
않으므로 기본 requirements 설치만으로 변환 환경이 완성됐다고 가정하지 마십시오.

~~~powershell
cd D:\Squid-Robot-Delivery-20260922\5_DepthLearning
python -m pip install -r requirements.txt
python -m pip install numpy esp-ppq
~~~

다음 예제는 기존 v1을 덮어쓰지 않고 비어 있는 **새 출력 폴더**를 사용합니다.

~~~powershell
python -B -m learning.model_based.train --train-csv ../4_PC_Console/target-pi-build/release/squid-logs/control-1791351717.csv ../4_PC_Console/target-pi-build/release/squid-logs/control-1791354652.csv --test-csv ../4_PC_Console/target-pi-build/release/squid-logs/control-1791358193.csv --output-dir artifacts/depth_model_based_new --epochs 400 --seed 42
python -B -m learning.model_based.export --model-dir artifacts/depth_model_based_new
python -B -m learning.model_based.verify --model-dir artifacts/depth_model_based_new
~~~

현재 v1의 학습 408개·검증 140개는 앞의 두 CSV에서 구간별로 분리한 표본이며,
세 번째 CSV 315개는 별도 시험 표본입니다. 학습과 시험 파일은 겹치면 안 됩니다.
현재 loader는 학습 자료의 세 구간 이상, 학습 100개·검증 30개 이상 표본이 필요합니다.
비어 있지 않은 출력 폴더에는 학습을 거부합니다.

학습은 0.5초 후 수심·속도 변화를 배우고, 기본 출력과 보정을 유지한다고 가정한 3초
예측 비용으로 -20~20 후보를 비교합니다. 실험으로 측정한 최적 보정값은 아닙니다.
동역학 검증 또는 교사 라벨 조건을 통과하지 못하면 정책 생성을 중단하고 실패 보고서를 남깁니다.

## 6. 제어기 연결 / 빌드 / 업로드

제어기 main/CMakeLists.txt의 depth_model_dir가 현재 artifacts/depth_model_based_v1/을 선택합니다.
다른 모델을 적용하려면 검증된 결과 폴더로 설정을 바꾸고 다시 빌드해야 합니다.
생성 헤더의 SQUID_DEPTH_MODEL_BASED가 입력 변환과 가드를 활성화합니다.
학습·변환만으로 로봇의 모델이 바뀌지는 않습니다.

ESP-IDF 5.4.4 환경에서:

~~~powershell
cd D:\Squid-Robot-Delivery-20260922\1_ESP32_MainController
idf.py -B build-codex build
~~~

현재 빌드 결과는 build-codex/squid_robot2.bin입니다.
OTA는 [주 제어기 README](../1_ESP32_MainController/README.md)를 참고하고,
로봇을 정지시킨 상태에서 별도로 수행하십시오.

## 7. PC 데이터 수집 / 최근 시험

사용 콘솔은 ../4_PC_Console/target-pi-build/release/squid-console2.exe입니다.
실행 작업 폴더 아래 squid-logs/에 raw-*.log를 기록하고,
수동 콘솔에서는 **F4**로 control-*.csv 저장을 켜거나 끕니다.
기록은 [CTRL] 도착 기준 약 0.5초이며 유실·잘못된 프레임이 있을 수 있습니다.
콘솔과 별도 수집 프로그램이 같은 COM 포트를 동시에 열 수는 없습니다.

2026-10-07에 실행한 순차 시험:

~~~text
30cm 30초 → 40cm 30초 → 50cm 30초 → l0 상향
→ 15cm 이하 2초 확인 → s 정지 → 정지 상태 확인
~~~

목표 오차가 커도 각 구간은 30초 기록했습니다. 시험 실행부에서는 수심 80cm 도달,
센서 이상 또는 유효 수심 수신 2초 초과 공백이면 중단하도록 했습니다.
상향 제한 시간은 60초입니다. 이것은 **해당 PC 시험의 조건**이며,
펌웨어에 80cm 제한이나 자동 15cm 종료 기능을 추가한 것은 아닙니다.
재사용 자동 시험 프로그램은 아직 저장소에 저장되어 있지 않습니다.

| 기록 | 용도 |
|---|---|
| control-1791369511.csv / raw-1791369511.log | 30cm 단일 30초 시험 |
| control-1791370021.csv / raw-1791370021.log | 30/40/50cm 순차 시험, 전체 220개 표본 |
| artifacts/depth_sequence_1791370021.png | 순차 시험·상향·정지 수심 그래프 |

CSV·통신 로그는 ../4_PC_Console/target-pi-build/release/squid-logs/에 있습니다.
최근 수집 데이터는 현재 v1 모델에 아직 재학습하지 않았습니다.

## 8. 기존 방식과 주의사항

learning.train은 과거 clip(Kp*오차 - Kd*속도, -20, 20) PD 규칙을 모방하며,
learning.export_espdl도 그 규칙으로 라벨을 재생합니다.
현재 모델에는 반드시 **learning.model_based.export**를 사용하십시오.

현재 모델은 오차 절댓값 >15cm, 속도 절댓값 >5cm/s 또는 정규화 입력 절댓값 >6이면
보정 0으로 복귀합니다. 정상 입력 5프레임을 모으기 전에도 보정은 0입니다.
추론은 500ms마다 하며 PID는 각 새 수심 측정에서 실행합니다.

별도 시험 자료의 0.5초 후 수심 변화 예측 MAE 약 0.351cm와 보정 모방 오차 약 5.70포인트는
**실제 목표 수심 유지 오차가 아닙니다**. 동역학 자료는 500ms 출력 스냅샷이며 사이의 PID
변화가 생략되어 있습니다. 전진 중 성능, PID 변경 이후 분포와 안정성은 추가 시험이 필요합니다.
실제 수심 시험은 수행했지만 동일 조건의 반복 비교로 성능 개선이 확정된 것은 아닙니다.
정지 명령은 목표 유지를 해제하므로 정지 후에도 수심이 변할 수 있습니다.
현장 감시와 즉시 회수가 가능한 환경에서만 시험하십시오.
