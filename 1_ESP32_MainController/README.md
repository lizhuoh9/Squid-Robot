# Squid-Robot 主控 — ESP-IDF 原生版

这是 `ESP32/`（Arduino 框架，V7）主控固件的 **完全原生 ESP-IDF 重写**。
逻辑、协议、引脚、控制算法与原版保持一致，底层全部换成 ESP-IDF API。

- 框架：ESP-IDF **v5.3**
- 目标芯片：**ESP32-S3**（`idf.py set-target esp32s3`）
- 语言：C++（无 Arduino 依赖）

## 目录结构

```
ESP32-IDF/
├── CMakeLists.txt          顶层工程
├── sdkconfig.defaults      默认配置（目标/分区/FATFS/HTTP/控制台）
├── partitions.csv          双 OTA 槽分区表
└── main/
    ├── CMakeLists.txt      源文件与依赖组件
    ├── Board.h             引脚 + 协议 + 执行器位掩码（原 Protocol.h）
    ├── Sys.h               millis/delay/delayMicroseconds
    ├── Output.h/.cpp       调试输出三路分发（原 Print/TeeStream/Serial/HC12）
    ├── Console.h/.cpp      USB/UART0 命令输入（原 Serial.read）
    ├── CH9434A.*           SPI 转 4 路 UART（原生 spi_master）
    ├── KalmanFilter.*      三状态深度滤波（纯数学）
    ├── DepthSensorManager.*  MS5837 深度（原生 I2C）
    ├── UltrasonicManager.*   三路超声波
    ├── ImuManager.*          EBIMU-9DOFV6
    ├── MotionLink.* / StatusDisplay.*   与 Minima 的 UART1 收发
    ├── ForwardControl.* / LeftTurnControl.* / RightTurnControl.*
    ├── DepthController.* / AutoNavigator.*
    ├── SensorHub.*          汇总显示 + 电池 ADC（原生 adc_oneshot）
    ├── SdCard.*             SD 卡挂载（sdspi + FATFS → /sdcard）
    ├── SDLogger.*           session 日志（POSIX 文件 API）
    ├── CommandHandler.*     命令解析（std::string；HC-12 UART2）
    ├── OtaManager.*         WiFi / NTP / HTTP 文件管理 / 固件 OTA
    └── main.cpp             app_main（原 ESP32.ino 的 setup/loop）
```

## 编译 / 烧录 / 监视

在 **ESP-IDF 5.3 PowerShell**（桌面快捷方式，已激活环境）里：

```bash
cd C:\Users\TKAET\Desktop\TKAET\Squid-Robot-main\ESP32-IDF
idf.py set-target esp32s3      # 只需第一次
idf.py build
idf.py -p COM<x> flash monitor  # 换成你的串口号
```

## 与 Arduino 版的行为差异（重要）

1. **OTA 无线烧录方式变了**。原版用 `ArduinoOTA`（espota 协议，Arduino 专有）。
   原生版改为 **HTTP 固件上传**，等价能力：

   ```bash
   idf.py build
   curl -X POST --data-binary @build/squid_robot.bin http://<机器人IP>/ota
   ```

   上传成功后设备自动切换 OTA 分区并重启。

2. **WiFi 配网**。与当前运行的 Arduino 固件一致，优先用 NVS 里保存的凭据连接（10s）。
   **首次**没有凭据时：复制 `main/OtaConfig.h` 为 `main/OtaConfig.local.h`，把
   `WIFI_FALLBACK_SSID/PASS` 改成你的 WiFi，编译烧录连上一次后会自动存进 NVS，
   之后就不需要了。（Arduino 版里那个已停用的阻塞式 Captive Portal 未移植。）

3. **控制台串口**。默认走 **UART0**（每块 S3 都有物理 TXD0/RXD0）。若你的板子用
   USB-Serial-JTAG 当串口，在 `idf.py menuconfig → Component config → ESP System
   Settings → Channel for console output` 改成 USB Serial/JTAG。

4. 其余（HC-12 无线、SD 20Hz 训练日志、SD 文件管理网页 `http://<IP>/`、
   深度/超声波/IMU、自动避障、定深 PID、急停气压平衡）行为与原版一致。

## 网页接口

控制只走 USB 串口和 HC-12；网页控制台（日志 + 发命令）已于 2026-09-19 删除，
网页只保留 SD 文件管理和 OTA。

- `GET  /` 或 `/console`     SD 文件管理页面（浏览 / 下载 / 打包 zip / 上传 / 删除）
- `GET  /files?path=/`       目录 JSON
- `GET  /file?path=/x/y`     下载
- `DELETE /file?path=/x/y`   递归删除
- `POST /upload?path=/dir`   multipart 上传
- `POST /ota`                固件上传

## 备注

- `Minima/`、`Minima_Bridge/` 是 Arduino UNO R4，仍在原目录，未改动（非 ESP32，不跑 ESP-IDF）。
- Flash 大小默认按 8MB 配置（`sdkconfig.defaults`），按实际模组可在 menuconfig 调整。
