# Squid Robot 주 제어기 — ESP-IDF / ESP-DL

## 1. 개요

1_ESP32_MainController/는 ESP32-S3에서 센서 처리, 수심 PID, 학습 보정 추론,
전진·회전 의도 생성, HC-12 명령 수신과 WiFi OTA를 수행합니다.
밸브의 반복 타이밍은 2_Minima_Actuator/가 담당합니다.

- 현재 빌드 환경: ESP-IDF **5.4.4**, 대상 esp32s3, C++
- 현재 학습 모델: ../5_DepthLearning/artifacts/depth_model_based_v1/
- 기본 PID: **Kp=10 / Ki=1.2 / Kd=15**, 상황별 배율 유지
- 출력: PID 기본 ±80, 학습 보정 ±20, 최종 ±100
- SD 관련 소스는 남아 있지만 현재 실험 데이터는 SD 없이 PC에 기록

이전 제안의 Ki=0.6은 적용되어 있지 않으며 소스상 Ki는 1.2입니다.
2026-10-07 대화에서 확인한 업로드·순차 시험은 20/1.2/30 버전입니다.
문서 수정 중 확인한 이후 10/1.2/15 소스의 실제 업로드 여부는 이번 작업에서 확인하지 않았습니다.
현재 모델은 기본 PID 이득을 낮추기 전에 학습한 모델입니다.

## 2. 디렉터리 구조

~~~text
1_ESP32_MainController/
├── CMakeLists.txt              ESP-IDF 프로젝트
├── sdkconfig / sdkconfig.defaults
├── partitions.csv             factory + ota_0 + ota_1, 각각 0x180000
├── dependencies.lock          해석된 의존성
├── managed_components/        ESP-DL 등 의존성 소스
├── build-codex/                현재 빌드 결과
└── main/
    ├── CMakeLists.txt          소스 등록·모델 폴더·바이너리 포함
    ├── idf_component.yml      ESP-DL 의존성 선언
    ├── depth_model.compile_req.yml  필요한 ESP-DL 연산
    ├── main.cpp               초기화·메인 루프·motion 작업·제어 기록
    ├── Board.h / Calibration.h  핀·프로토콜·방향 매핑
    ├── DepthController.*      PID·속도 제한·수동 부력·기압 평형
    ├── DepthResidualModel.*   ESP-DL 입력 구성·추론·가드
    ├── DepthSensorManager.* / KalmanFilter.*  수심 측정·필터
    ├── ForwardControl.* / TurnControl.* / AutoNavigator.*
    ├── MotionLink.* / MotionLock.* / StatusDisplay.*
    ├── SensorHub.* / UltrasonicManager.* / ImuManager.* / CH9434A.*
    ├── CommandHandler.* / Console.* / Output.*
    ├── OtaManager.* / OtaConfig.h / OtaConfig.local.h
    └── SdCard.* / SDLogger.*   기존 선택적 SD 기능
~~~

## 3. 수심 제어와 학습 모델 연결

DepthController.cpp는 새 수심 측정마다 PID와 속도 제한을 계산합니다.
기본값을 기준으로 오차·변화율·속도에 따른 적응형 배율을 적용하므로
실제 Kp/Ki/Kd는 항상 10/1.2/15로 고정되는 것은 아닙니다.

DepthResidualModel.cpp는 500ms마다 5개 프레임을 모으고 보정값을 추론합니다.
입력은 목표-현재 수심, 속도, 가속도, PID 기본 출력, 기본 출력으로 계산한 PWM입니다.
정규화는 INT8 양자화 전에 수행합니다. 작은 보정 모델만 로봇에서 실행하며,
동역학 모델과 보정 후보 탐색은 PC 학습용입니다.

오차 절댓값 >15cm, 속도 절댓값 >5cm/s, 학습 범위 밖의 입력, 센서 이상이나
불충분한 입력 이력에서는 보정 0으로 복귀합니다.
기본 PID의 I항은 사용하지만 추가 누적 오차 딥러닝 항은 사용하지 않습니다.
기존 ±0.2cm 데드존, 최종 출력 절댓값 8 미만의 펌프 정지 규칙도 유지됩니다.

main/CMakeLists.txt의 depth_model_dir가 모델 폴더를 지정합니다.
현재 폴더의 depth_model_espdl.espdl과 DepthResidualModelConfig.h는 한 쌍입니다.
모델 파일 또는 헤더를 따로 교체하지 마십시오.
예전 PD 모델은 ../5_DepthLearning/artifacts/depth_model_pd_restore_1790936043/에 보존됩니다.

## 4. 빌드 / 업로드

설치된 ESP-IDF 5.4.4 환경을 활성화한 PowerShell에서:

~~~powershell
cd D:\Squid-Robot-Delivery-20260922\1_ESP32_MainController
idf.py -B build-codex build
~~~

빌드 결과 경로는 build-codex/squid_robot2.bin입니다. 이 문서 작업에서는 빌드하지 않았으므로,
기존 바이너리가 현재 소스의 PID값과 일치한다고 가정하지 마십시오.
COM4는 PC의 HC-12 Bridge 연결이며 ESP32 USB 플래시 포트가 아닙니다.
현재 밀폐된 로봇은 WiFi HTTP OTA로 업로드합니다. 사전 제작 바이너리나
다른 build 폴더의 파일을 현재 빌드 결과로 혼동하지 마십시오.

로봇을 정지시키고 연결 대상 IP·MAC을 확인한 뒤, 필요할 때만 다음을 실행합니다.

~~~powershell
curl.exe --noproxy "*" --header "Content-Type: application/octet-stream" --header "Expect:" --data-binary "@build-codex/squid_robot2.bin" http://192.168.0.183/ota
~~~

현재 알려진 IP는 192.168.0.183이며 DHCP로 바뀔 수 있습니다.
로봇 MAC은 80:b5:4e:ce:ec:88입니다. 로봇은 2.4GHz ssbrl에 연결하고,
같은 LAN에 도달 가능한 PC는 ssbrl5G에서도 업로드할 수 있습니다.
업로드 성공 후 자동 재부팅합니다. HTTP 접속 복구 확인만으로 센서·제어 전체가
정상임을 보장할 수는 없으므로 별도 수신·동작 확인이 필요합니다.
자세한 네트워크·복구 경로는 [전체 안내](../Docs/README.md)를 참고하십시오.

## 5. PC 콘솔과 데이터 기록

현재 사용 콘솔: ../4_PC_Console/target-pi-build/release/squid-console2.exe.
Bridge USB는 115200 baud, HC-12 무선은 9600 baud입니다.
최근 시험에서 Bridge는 COM4였지만 연결할 때 확인하십시오.

제어 기록 [CTRL]은 main.cpp의 CONTROL_TELEMETRY_INTERVAL_MS=500에 따라 전송됩니다.
PC 콘솔에서 F4로 CSV 저장을 켜며, 실행 작업 폴더의 squid-logs/에 저장됩니다.
raw-*.log는 통신 원본, control-*.csv는 제어 표본입니다.
SD 카드가 없어도 이 수집 경로는 동작합니다.
CSV 목록과 학습 방법은 [5_DepthLearning README](../5_DepthLearning/README.md)를 참고하십시오.

## 6. 명령과 시험 종료

| 명령 | 의미 |
|---|---|
| l30 / l40 / l50 | 목표 수심 30 / 40 / 50cm 유지 |
| l0 | 목표 해제 후 수동 상향 지속; 다음 명령이 필요 |
| s | 긴급 정지·목표 해제·5초 기압 평형 및 명령 잠금 |
| j / k | 방향 매핑에 따른 수동 하강 / 상향 토글 |
| g | 센서 화면 표시 |
| c | 현재 수심을 영점으로 보정; 시험 중 임의 실행 금지 |
| hb | 원격 연결 heartbeat |
| md / mt | DEBUG / TEST; TEST에서는 WiFi 사용 불가 |

l0는 15cm에서 자동으로 멈추는 명령이 아닙니다.
최근 순차 시험의 30/40/50cm 각 30초, 80cm 한계, 15cm 이하 2초 확인 후 정지는
PC 실행부에서 적용한 조건입니다. 재사용 자동 시험 스크립트는 저장되어 있지 않습니다.
펌웨어의 수심 제어 상한 100cm와 해당 시험의 80cm 한계를 혼동하지 마십시오.

## 7. 웹 인터페이스 / 주의사항

제어 명령은 USB 또는 HC-12로 전달합니다. HTTP 페이지는 파일 관리와 OTA용이며
시리얼 명령 입력창이 아닙니다.

| 인터페이스 | 역할 |
|---|---|
| GET / 또는 /console | 기존 SD 파일 관리 페이지 |
| GET /files?path=/ | 파일 목록 |
| GET /file?path=/x/y | 파일 다운로드 |
| DELETE /file?path=/x/y | 삭제 |
| POST /upload?path=/dir | 파일 업로드 |
| POST /ota | 펌웨어 업로드 |

방향은 Calibration.h의 BUOY_SINK / BUOY_RISE 매핑을 기준으로 판단합니다.
시리얼 포트는 콘솔과 별도 제어 프로그램이 동시에 열지 않습니다.
s 이후에도 관성·부력 때문에 수심이 변할 수 있으므로 현장 감시가 필요합니다.
PID 또는 밸브 동작을 변경하면 기존 모델의 입력 분포·예측 효과도 변하므로,
기본 제어를 시험하고 변경된 조건의 데이터를 모아 재학습 여부를 판단하십시오.
