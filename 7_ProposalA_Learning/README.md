# 7. Proposal A Learning — 데이터 수집과 Hybrid/MPC 시험 학습

기존 PID+딥러닝과 Proposal A 프로젝트를 유지하면서, 새로운 **동역학 학습 → MPC 교사 → 작은 보정 네트워크** 방식에 필요한 데이터를 수집하는 별도 프로젝트입니다.

SD 없는 RAM 기록 기능과 함께, 2026-10-08의 30/40/50cm CSV 세 개로 동역학 모델 3개, MPC 교사, 학생 네트워크 및 ESP-DL INT8 모델을 생성했습니다. 제어 코드에 effort 보정 경로도 연결했습니다. **검증 기준은 미달이지만 사용자 최종 승인으로 `kEnabled=true`인 실험용 빌드를 만들었습니다.** 기존 잔차 모델을 재사용하지 않습니다. 성능 개선은 아직 보장되지 않습니다.

## 폴더 구성

```text
7_ProposalA_Learning/
  firmware/                  Proposal A 소스 복사본 + RAM 기록 기능
    main/RamCapture.*        20Hz 기록 / HTTP 다운로드
    build-recording/         별도 ESP32-S3 빌드 결과
  pc/collector.py            기록 시작·상태 조회·다운로드·CSV 변환
  tests/test_collector.py    오프라인 데이터 검증 테스트
  tests/test_learning.py     학습 분리 / 교사 / 실험용 승인 상태 검증
  learning/                 pipeline.py / export_espdl.py
  data/                      새 수집 데이터 저장 위치
  artifacts/                 기존 파일 보존 검증 자료
```

`1_ESP32_MainController`, `5_DepthLearning`, `6_DepthHold_ProposalA`의 소스·모델·빌드는 변경하지 않습니다. Proposal A 제어 상수는 동일하고, 추가된 학생 보정은 ±0.3cm/s² 제한·입력 보호 조건을 유지합니다. 이전 수집용 바이너리와 main 소스는 `artifacts/pre_learning_recorder/`에 보관했습니다.

## 기록 방식과 제한

- SD 카드를 사용하지 않습니다. 50ms 간격으로 RAM에 기록하고, 수면에서 Wi-Fi로 PC에 다운로드합니다. HC-12에 새로운 고속 로그를 전송하지 않습니다.
- 20Hz는 **제어 상태 스냅샷 주기**입니다. 현재 센서의 새 수심 측정은 코드상 약 12Hz이므로 동일 측정값이 반복될 수 있습니다. `depth_fresh`, `depth_sequence`, `depth_age_ms`로 이를 구분합니다. 실기 측정률은 검증이 필요합니다.
- 1회 1~120초를 요청할 수 있습니다. 기본 요청은 60초입니다. 60초 데이터는 약 67,200바이트, 120초는 약 134,400바이트의 RAM을 사용합니다. 별도로 64KiB 여유 힙과 연속 메모리를 확인하고, 부족하면 HTTP 507로 거부합니다. 실제 로봇에서 가능한 시간은 아직 확인하지 않았습니다.
- 기록 지연은 실제 `elapsed_s`와 `missed_slots`로 남깁니다. 빠진 샘플을 만들어 넣지 않습니다.
- 다운로드 후에도 RAM 데이터는 유지됩니다. 확인 후 명시적으로 `clear`해야 다음 기록을 시작할 수 있습니다. 재부팅·전원 차단 시 데이터는 소실됩니다.
- CRC32, 레코드 수, 시간 순서, 스키마와 주요 값의 유효성을 확인하고 저장합니다. 같은 이름의 파일은 덮어쓰지 않습니다.

### 저장 필드

| 구분 | 필드 | 의미 |
| --- | --- | --- |
| 시간 / 갱신 | `record_index`, `elapsed_s`, `depth_age_ms`, `depth_sequence`, `depth_valid`, `depth_fresh` | 기록 시작 이후 실제 시간과 센서 갱신 여부 |
| 수심 상태 | `depth_cm`, `raw_depth_cm`, `depth_speed_cm_s`, `depth_accel_cm_s2` | 필터 수심 / 원시 수심 / 추정 속도·가속도 |
| 목표 / 제어 | `target_valid`, `target_depth_cm`, `proposal_desired_accel_cm_s2`, `proposal_bias_cm_s2`, `requested_duty` | Proposal A의 목표와 내부 제어값 |
| 실행 명령 | `pwm_commanded`, `direction_commanded`, `valve_mask_estimated` | ESP가 보낸 PWM·방향 명령과 추정 밸브 마스크 |
| 보조 상태 | `battery_window_v`, `forward_active`, `balance_active`, `output_saturated` | 전압 창 값과 제어 상태 |

PWM은 0~255 명령값, 방향은 기존 펌웨어의 방향 코드, duty는 -1~1입니다. **실제로 실행된 PWM·밸브 피드백이나 유량은 아닙니다.** 밸브 마스크는 명령으로 추정하며 균압 교대 위상도 실제 핀 상태와 다를 수 있습니다. 전압은 기존 SensorHub의 **3초 최대값 창**이지 순간 전압이 아닙니다. 이후 정확한 동역학 학습을 위해 실제 실행기 피드백과 전압 측정 개선을 검토해야 합니다.

## PC 사용 방법

Python 표준 라이브러리만 사용합니다. 추가 패키지 설치가 필요 없습니다. 아래 명령은 `7_ProposalA_Learning` 폴더에서 실행합니다. IP는 실제 로봇 IP로 바꾸십시오.

```powershell
python -B pc/collector.py --url http://192.168.0.183 status
python -B pc/collector.py --url http://192.168.0.183 start --duration 60
```

로봇은 별도 기존 콘솔로 수동 제어합니다. 이 도구는 시리얼 포트를 열지 않아 기존 콘솔과 포트가 충돌하지 않습니다. 수면에서 기록을 시작하고, 사람이 시험·회수를 감독한 뒤 전원을 유지한 채 다운로드합니다.

```powershell
python -B pc/collector.py --url http://192.168.0.183 download --output-prefix data/trial-001
```

`trial-001.sqdcap` 원본, `trial-001.csv` 데이터, `trial-001.json` 설정·검증 정보를 저장합니다. 출력 이름을 생략하면 UTC 시각 기반 이름으로 `data/`에 저장합니다. 수중 Wi-Fi 단절 시 RAM 기록은 계속되지만, 수면에서 연결 회복 후 다운로드해야 합니다.

시작 후 연결 회복까지 기다려 다운로드하는 명령도 있습니다.

```powershell
python -B pc/collector.py --url http://192.168.0.183 record --duration 60 --wait-surface 120
```

`record`는 로봇을 조종하지 않습니다. 대기 제한을 넘으면 수동 다운로드 안내와 함께 종료하고 RAM 데이터는 남깁니다. 상태 조회를 위한 HTTP 요청만 보내므로, 수중 기록 중 PC 연결이 필수는 아닙니다.

```powershell
python -B pc/collector.py --url http://192.168.0.183 stop
python -B pc/collector.py --url http://192.168.0.183 clear
python -B pc/collector.py decode data/trial-001.sqdcap --output-prefix data/redecoded-001
```

**`stop`은 기록만 멈춥니다. 로봇의 운동·펌프를 정지시키거나 부상시키지 않습니다.** 시간 만료도 마찬가지입니다. `clear`는 기록 종료 후 RAM 데이터만 삭제합니다. 이 도구에는 자동 수심 명령, 5초 이상 센서 오류 시 자동 회수, 80cm 제한, 종료 수심 15cm 자동 부상 기능이 없습니다. 기존 제어기의 보호 기능만으로 시험 안전성을 보장할 수 없으므로 사람이 상태를 감독하고 독립적인 회수 수단을 준비해야 합니다.

## 빌드와 오프라인 검증

ESP-IDF v5.4.4 환경에서:

```powershell
cd firmware
idf.py -B build-recording build
```

기존 수집용 바이너리: `firmware/build-recording/squid_robot_a_recorder.bin`. 학습 연결 버전은 아래 명령으로 별도 빌드합니다. 이미 설치된 ESP-DL 구성요소를 `firmware/local_components`에 복사해 사용하므로 기존 프로젝트 의존성은 수정하지 않습니다.

```powershell
$env:IDF_COMPONENT_MANAGER='0'
idf.py -B build-learning build
```

학습 연결 앱: `firmware/build-learning/squid_robot_a_recorder.bin`. 상태 API의 `firmware=hybrid_mpc_v1`, `student_embedded=true`, `student_enabled=true`로 구분합니다. `student_ready=true`는 모델 로딩과 추론 작업 생성 성공을 뜻하며, 실기 개선 검증을 뜻하지 않습니다. 업로드 완료 여부는 별도 `artifacts/hybrid_mpc_v1/deployment_report.json`을 확인하십시오.

프로젝트 폴더에서:

```powershell
python -B -m unittest discover -s tests -v
```

테스트는 로봇에 연결하지 않습니다. CRC 오류, 잘린 데이터, 스키마·시간·갱신 플래그, 저장과 덮어쓰기 방지를 확인합니다. 기존 파일의 SHA-256 목록은 `artifacts/preservation_manifest.json`에 보관합니다.

## 시험 학습과 한계

`learning/pipeline.py`는 세 CSV에서 과거 3초와 미래 3초 전체가 분리 블록 안에 포함되도록 학습/검증/시험을 나눕니다. 센서 값은 인과적 이전 값 유지로 100ms 정렬하며 PWM 명령은 구간별 시간 가중 평균을 사용합니다. 물리식과 작은 MLP로 수심·속도·숨은 부력 상태를 추정하고 3초 다단계 손실로 학습합니다.

교사는 4초·4개 구간의 보정 계획 81개를 탐색하며 Proposal A PD/적분, 최소 PWM 펄스, 방향 변경 시 200ms 지연을 모델 내에서 다시 계산합니다. 시작 펄스 위상과 실제 실행기 상태는 측정되지 않아 가정하며, 실제 Kalman 관측기 전체를 재현한 시뮬레이션은 아닙니다. 11개 파라미터 변형도 실제 11개 환경 검증을 의미하지 않습니다.

학생은 34입력 → 32 → 16 → 1 MLP입니다. 출력은 ±0.3cm/s² effort 보정이며 raw PWM이 아닙니다. 앙상블 불확실성은 오프라인 교사에서만 계산하고 보드에는 학생 하나만 들어갑니다. 보드 경로는 유효값/입력범위/3초 이력/250ms 결과 수명/수동·균압 제외/세대 번호로 오래된 결과 차단을 적용합니다. 활성화 시 별도 저우선순위 작업에서 추론합니다.

**현재 결과:** 3초 수심 예측 MAE는 3.71cm(일정 속도 기준 7.69cm)입니다. 다만 학생 모델의 모델 기반 제어 비용 개선은 약 1.1%로 5% 기준을 넘지 못했고, 독립 실험·개방루프·실기 검증도 없습니다. `training_report.json`의 실패 게이트는 그대로 보존하고 사용자의 실험용 활성화 승인은 배포 보고서에 별도로 기록합니다. 이 예측 오차를 실제 수심 유지 오차로 해석하면 안 됩니다.

```powershell
# 프로젝트 폴더에서 실행. 추가 로봇 명령은 보내지 않습니다.
python -B learning/pipeline.py --epochs 80
python -B learning/export_espdl.py
```

출력: `artifacts/hybrid_mpc_v1/`의 동역학 앙상블, 교사 정답 CSV, 학생 PT/ONNX/ESP-DL, 정규화 헤더와 평가 JSON입니다. ESP-DL과 헤더는 반드시 함께 갱신하고 재빌드하십시오. exporter는 자동으로 보정을 활성화하지 않습니다.

## 다음 검증 단계

1. RAM 기록 부하·힙 여유·실제 센서 주기를 확인하고, Proposal A 수심 유지와 별도 승인된 개방루프 펄스 시험을 진행합니다. 최소 동작 PWM도 측정합니다. 30~60분 총 데이터는 짧은 세션 여러 개로 모읍니다.
2. 데이터를 더 모아 시험 동역학 모델을 재학습합니다. 세션 전체를 학습/검증에서 분리하고, 현재 3.71cm 예측 오차를 충분히 줄입니다.
3. 실제 PWM/방향/밸브 피드백과 관측기 지연을 반영해 교사 시뮬레이션을 보완하고 불확실성 기준을 검증합니다.
4. 실험용 보정은 사람이 감독하는 짧은 시험에서 Proposal A 단독과 비교합니다. 악화되면 즉시 중단하고 보정을 비활성화한 버전으로 돌아갑니다. 자동 수중 시험은 이번 활성화·업로드 작업에 포함되지 않습니다.
5. 앙상블 분산은 PC 교사에서 사용합니다. 단일 학생 모델이 보드에서 그 분산을 직접 계산할 수는 없으므로 입력 범위·유효성·신뢰도 등의 별도 보호 조건을 설계합니다. 이상 시 보정은 0으로 돌아가되, Proposal A 단독 운전 자체의 안전성도 따로 검증합니다.
6. 도메인 랜덤화와 새 실험 데이터의 교사 재라벨링을 반복합니다. 동일한 시험 조건에서 Proposal A 단독보다 확실히 나은 경우에만 보정을 채택합니다.
