# Minima 执行器固件（已固定）

**这块板的固件逻辑已冻结，日常开发只烧 ESP32。** 它是纯执行端：收到什么就把引脚写成什么，
不含任何运动时序。所有时序（前进/转向阀门切换周期、平衡交替、换向保护、上浮下沉阀门组合、
PID 力度）都在 ESP32 `ESP32-IDF/` 里，改那边即可，不需要重烧这块。

## 它只做四件事

1. **执行掩码**：`CMD_SET_ACTUATORS` 的 16 位掩码直接写到前进/转向的泵和阀。
2. **执行浮沉**：`CMD_SET_BUOYANCY` 的 `data0` = 阀门位（bit0=阀E，bit1=阀F），`data1` = 泵 PWM
   （0~255）。阀门照写，泵用 20ms 周期的软件 PWM 生成波形——这是唯一留在本地的"时序"，
   因为波形必须在本地产生。
3. **链路失效保护**：超过 1 秒收不到 ESP32 的有效帧，立即停掉全部泵阀；收到新帧自动恢复。
   ESP32 每 200ms 会重发一次当前状态，正常工作时不会误触发。
4. **状态回传**：每 500ms 回一帧运动状态（含实际执行掩码），每 1s 回一次心跳
   （含累计坏帧数、上一秒最长 loop），供 ESP32 核对链路质量。

## 版本与校验

- 版本号在 `Minima.ino` 的 `MINIMA_FW_VERSION`，开机会打印到 USB 串口。
- `prebuilt/Minima.ino.bin` 是与本目录源码一致的编译产物，`.sha256` 为其校验和。
  想确认板上固件是不是这一版，看开机横幅的版本号即可。

## 烧录方法（仅在必须时）

执行器和 ESP32 共用一条 USB，先把硬件开关拨到执行器侧，它会枚举成一个新的 COM 口
（与遥控发射端同为 "Arduino UNO R4 Minima"，按 USB 序列号区分：执行器
`35181E0859323335CA2C33334B57305C`）。两块同时在线时 `arduino-cli upload` 会因为
"More than one DFU capable USB device" 失败，改用：

```
arduino-cli compile --fqbn arduino:renesas_uno:minima --output-dir prebuilt .
dfu-util --device 0x2341:0x0069,:0x0369 -S <执行器序列号> -D prebuilt/Minima.ino.bin -a0 -Q
```

板子在正常运行态时 dfu-util 会报 "No DFU capable USB device"，需要先用 1200bps 触碰
该 COM 口让它进 bootloader，再执行 dfu-util。
