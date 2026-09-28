# Squid Robot — Source Delivery

Package date: 2026-09-22

The source in this package is **exactly what is running on the hardware right now**.
All three boards were flashed from these files and verified.

| Board | Firmware version (printed at boot) |
|---|---|
| ESP32-S3 main controller | `Squid-ESP32 v2.0-BASE` |
| Minima actuator | `Minima-Executor v2.1` |
| Minima remote transmitter | `SQUID-HC12-BRIDGE v2.0` |

---

## 1. Network configuration — read this first

The ESP32-S3 is **sealed inside the hull and the USB port is not reachable**. OTA over
WiFi is the only way to update it. There are therefore two independent access paths, and
both are active at the same time (the radio runs in AP+STA mode).

### Path A — lab WiFi (preferred, use this normally)

The robot joins this network automatically at boot:

| | |
|---|---|
| SSID | **`ssbrl`** |
| Password | **`brl12345`** |
| Band | 2.4 GHz |
| Robot IP (current) | **`192.168.0.183`** (DHCP — may change) |

**Do not use `ssbrl5G`.** The ESP32-S3 radio is 2.4 GHz only and cannot see 5 GHz networks.

Any computer on the lab network can flash the robot directly. The lab router does not use
client isolation, so this works from both the 2.4 GHz and the 5 GHz SSID, and the computer
**keeps its normal internet connection**:

```
curl -X POST --data-binary @squid_robot2.bin http://192.168.0.183/ota
```

The IP is assigned by DHCP and can change. To find it again, run this on any machine on the
lab network (the robot's MAC is `80:b5:4e:ce:ec:88`):

```
arp -a | findstr 80-b5-4e
```

Binding a static lease to that MAC in the router is recommended so the address never moves.

### Path B — the robot's own access point (fallback)

The robot always broadcasts its own network, whether or not the lab WiFi is reachable:

| | |
|---|---|
| SSID | **`SquidRobot`** |
| Password | **`squid12345`** |
| Robot IP | **`192.168.4.1`** (fixed, never changes) |

```
curl -X POST --data-binary @squid_robot2.bin http://192.168.4.1/ota
```

A computer connected to this AP has **no internet access** while connected, because the
robot does not route traffic. That is acceptable for the few seconds a flash takes; a
machine with a wired connection keeps internet over Ethernet and uses WiFi for the robot.

This path does not depend on any router or on anybody else's computer, which is why it
exists: it is the recovery path if the lab network changes, moves, or is unavailable.

### Connection order at boot

1. Try `ssbrl` (the SSID/password compiled into `OtaConfig.local.h`)
2. If that fails, fall back to the last network stored in NVS
3. If that also fails, open an open provisioning portal named `SquidRobot-Setup`
   (connect a phone, any web page redirects to the setup form, or open `http://192.168.4.1/`)

**The robot's own AP stays up during all of the above**, so `http://192.168.4.1/ota` works in
every state — including before the robot has ever been provisioned. A wrong WiFi password
therefore cannot lock you out.

### Changing the WiFi network

Edit `1_ESP32_MainController/main/OtaConfig.local.h`, rebuild, and flash. The credentials there are tried
first; the previously working network stays in NVS as a fallback, so a typo is not fatal.

---

## 2. What the system is

A biomimetic squid robot. It moves by pumping water: a pump pressurises a chamber and valves
switch on a fixed cadence, producing a jet. There are three independent actuator subsystems —
forward propulsion, turning, and buoyancy — each with its own pump and valve pair.

```
PC ──USB 115200── Minima transmitter ──HC-12 radio 9600── ESP32-S3 ──UART 2 Mbps── Minima actuator
  4_PC_Console       3_Minima_Transmitter                    1_ESP32_MainController              2_Minima_Actuator
                                                             |
                                    sensors: MS5837 depth / 3x ultrasonic / IMU / battery ADC / SD card
```

**Division of responsibility.** The ESP32 reads sensors, filters them, and decides *intent*
("start moving forward", "turn left", "ascend"). It never sends valve timing. The actuator
generates the valve cadence locally from parameters stored in its own EEPROM. This is
deliberate: radio jitter, dropped frames and host-side stalls cannot disturb the valve
rhythm, which is what the pneumatics actually care about.

---

## 3. Package layout

```
Squid-Robot-Delivery/
├── Docs/                      this file
├── 1_ESP32_MainController/    ESP32-S3 main controller   (ESP-IDF v5.3, C++)
├── 2_Minima_Actuator/         pumps and valves           (Arduino UNO R4 Minima)
├── 3_Minima_Transmitter/      handheld radio transmitter (Arduino UNO R4 Minima)
├── 4_PC_Console/              operator console           (Rust, Windows)
└── Prebuilt_Binaries/         ready to flash / ready to run
    ├── ESP32_MainController.bin      -> OTA target, see section 1
    ├── ESP32_bootloader.bin          -> only needed for a full USB flash
    ├── ESP32_partition-table.bin     -> only needed for a full USB flash
    └── PC_Console.exe                -> run this on the operator PC
```

Folders 2 and 3 are the same board model running different firmware. Folder 2 sits inside
the robot and drives the pumps and valves; folder 3 stays with the operator and relays
commands over the HC-12 radio.

## 4. Source file reference

### `1_ESP32_MainController/` — ESP32-S3 main controller (ESP-IDF v5.3, target esp32s3)

| File | Purpose |
|---|---|
| `main/main.cpp` | Startup, main loop, the 5 ms motion task, `ps` system status, diagnostics |
| `main/Board.h` | Pin map, UART/SPI/I2C assignments, protocol constants, parameter IDs |
| `main/Calibration.h` | **Direction mapping — the only place direction is configured** |
| `main/CommandHandler.*` | Command parsing for USB serial and HC-12, mode switching, failsafe |
| `main/MotionLink.*` | Framed protocol to the actuator; periodic intent refresh |
| `main/ForwardControl.*` | Forward propulsion intent (no timing — that lives in the actuator) |
| `main/TurnControl.*` | Left/right turn intent |
| `main/DepthController.*` | Depth-hold PID, manual buoyancy, pressure balance |
| `main/AutoNavigator.*` | Obstacle avoidance mode (`q`) |
| `main/SensorHub.*` | Sensor aggregation, console panel, compressed radio telemetry, battery |
| `main/DepthSensorManager.*` | MS5837 pressure sensor over I2C, with Kalman filtering |
| `main/UltrasonicManager.*` | Three ultrasonic rangefinders, round-robin over the CH9434A |
| `main/ImuManager.*` | EBIMU-9DOFV6 attitude sensor |
| `main/CH9434A.*` | SPI-to-quad-UART bridge driver (hosts the ultrasonics and the IMU) |
| `main/StatusDisplay.*` | Decodes status frames coming back from the actuator |
| `main/OtaManager.*` | WiFi (AP+STA), NTP, captive portal, HTTP file manager, OTA endpoint |
| `main/SDLogger.*`, `main/SdCard.*` | 20 Hz training log to SD card during TEST mode |
| `main/Output.*`, `main/Console.*` | Output routing: USB console and HC-12 radio |
| `main/MotionLock.*` | Mutex between the main loop and the motion task |
| `main/KalmanFilter.*` | Shared scalar Kalman filter |
| `main/OtaConfig.h`, `main/OtaConfig.local.h` | **WiFi credentials, hostname, AP name/password** |
| `partitions.csv` | `factory` + `ota_0` + `ota_1`, 1.5 MB each |
| `sdkconfig` | Committed so the build is reproducible |

### `2_Minima_Actuator/` — actuator (Arduino, `arduino:renesas_uno:minima`)

| File | Purpose |
|---|---|
| `Minima_Actuator.ino` | Frame receive, intent dispatch, status/heartbeat reporting |
| `MotionSeq.*` | The valve cadence state machines: forward, turn, buoyancy |
| `MotionControl.*` | Pin-level output and the software PWM for the buoyancy pump |
| `MotionParams.*` | Timing parameters with EEPROM persistence |
| `Protocol.*` | 8-byte framed protocol with CRC8 |
| `PinDefinitions.h` | Pump and valve pin assignments |

### `3_Minima_Transmitter/` — radio transmitter (Arduino, same board type)

| File | Purpose |
|---|---|
| `Minima_Transmitter.ino` | Transparent USB↔HC-12 bridge, command sequence numbers, ACK and retry |

### `4_PC_Console/` — operator console (Rust, GNU toolchain)

| File | Purpose |
|---|---|
| `src/main.rs` | CLI arguments, serial port auto-discovery, event loop, heartbeat |
| `src/app.rs` | Key handling, depth slider, log buffer |
| `src/parser.rs` | Parses telemetry frames and robot replies into dashboard state |
| `src/ui.rs` | Terminal dashboard rendering |
| `src/link.rs` | Serial port I/O and background reader thread |
| `src/picker.rs` | Serial port selection screen |
| `src/commands.rs` | On-screen command reference |
| `build.cmd` | Build helper that puts `dlltool` on PATH for the GNU toolchain |

### `Prebuilt_Binaries/`

`squid_robot2.bin` (ESP32 application), `bootloader.bin`, `partition-table.bin`,
`squid-console2.exe` (PC console, Windows x64).

---

## 5. Building and flashing

> **ESP-IDF cannot build under a path containing non-ASCII characters on Windows.** Its
> kconfig tooling decodes the project path with the system locale and crashes on Chinese
> path components. Extract this package to an ASCII-only path.

| Component | Build | Flash |
|---|---|---|
| ESP32-S3 | `idf.py build` | `curl -X POST --data-binary @build/squid_robot2.bin http://<robot-ip>/ota` |
| Actuator | `arduino-cli compile --fqbn arduino:renesas_uno:minima 2_Minima_Actuator` | `arduino-cli compile --upload -p <COM> ...` |
| Transmitter | same, with `3_Minima_Transmitter` | same |
| PC console | `build.cmd` | run `target/release/squid-console2.exe` |

### Two cautions when flashing the Arduino boards

**They are the same model and `arduino-cli board list` cannot tell them apart.** With both
connected, `upload` fails with *"More than one DFU capable USB device found"*. Address them by
USB serial number with `dfu-util` instead:

| Board | USB serial number |
|---|---|
| Actuator | `35181E0859323335CA2C33334B57305C` |
| Transmitter | `340E301358323033787A33334B57302F` |

```
arduino-cli compile --fqbn arduino:renesas_uno:minima --output-dir out 2_Minima_Actuator
dfu-util --device 0x2341:0x0069,:0x0369 -S <serial> -D out/2_Minima_Actuator.ino.bin -a0 -Q
```

**Flash the ESP32 before the transmitter** when updating both. The v2.0 transmitter sends
sequence-numbered commands (`w#7`); an older robot firmware rejects them and every command is
lost until the robot is updated. The reverse combination (new robot, old transmitter) is
backward compatible.

---

## 6. Commands

Accepted over USB serial and over the HC-12 radio. Everything is lower-cased and trimmed first.

| Command | Meaning |
|---|---|
| `w` | Forward propulsion on/off (toggle) |
| `a` / `d` | Turn left / right (one complete sequence per press) |
| `j` / `k` | Descend (intake) / ascend (exhaust) — see calibration below |
| `l<number>` | Hold depth at N cm; `l0` means surface and keep ascending |
| `s` | Emergency stop, then a 5 s pressure-balance lockout |
| `q` | Toggle manual / autonomous obstacle avoidance |
| `g` | Sensor panel on/off |
| `c` | Calibrate the current depth as zero |
| `md` / `mt` | DEBUG (WiFi on) / TEST (WiFi off, 20 Hz logging to SD) |
| `ps` | System status: task stacks, heap, actuator link quality, bad frame counters |
| `pget` / `pset <name> <value>` / `psave` | Read / set / persist the actuator timing parameters |
| `mark <note>` | Timestamped marker in the SD event log |
| `fs` / `fs<seconds>` | Remote-link failsafe: show / set / `fs0` disables |
| `h` | Full help |

The console additionally maps `PgUp`/`PgDn` and the arrow keys to the depth slider, `Space`
and `Esc` to emergency stop. The mouse wheel is deliberately ignored — see section 8.

---

## 7. Direction calibration

Which way the ultrasonics, the turn subsystem and the buoyancy subsystem actually point
depends on wiring and on how the pneumatics were assembled. Every direction decision in the
firmware goes through **`1_ESP32_MainController/main/Calibration.h`**. Change the switches there; do not
edit the comparisons inside `AutoNavigator`, `CommandHandler` or `DepthController`.

Measured values as of 2026-09-21:

| Switch | Value | Meaning |
|---|---|---|
| `FLIP_ULTRASONIC` | `true` | Left and right rangefinders are wired the other way round |
| `FLIP_TURN` | `false` | Turn directions are correct as wired |

**Buoyancy naming.** The protocol constants are named the opposite of their physical effect:
`BUOYANCY_ASCEND` closes both valves and runs the pump, which takes water in and makes the
robot **sink**. This is corrected once, in `Calibration.h`, as `BUOY_SINK` and `BUOY_RISE`.
Use those two names everywhere; do not write the raw protocol constants in control logic.

The same inversion previously made depth-hold drive away from its setpoint and made `l0`
("surface") sink instead. Both are fixed.

---

## 8. Design constraints that must not be broken

### The link to the actuator must be state-based, not event-based

`MotionLink::refresh()` re-asserts the current intent every 200 ms using the idempotent
`ACT_SYNC` action. Without that, a single lost frame desynchronises the system permanently:
the ESP32 believes the robot is moving forward while the actuator sits idle. Do not remove
the periodic re-assertion.

The actuator's link-timeout protection was removed on purpose. The two boards are connected
by a PCB trace and cannot lose each other; meanwhile that timeout destroyed the stored intent
along with the outputs, and nothing ever restored it, so it caused the very failure it was
meant to prevent.

### Propulsion may only be started when it is not already running

`ForwardSeq::start()` resets the valve phase. Calling it every loop iteration — which the
obstacle-avoidance mode used to do — keeps resetting the phase, the valves never switch, and
the chamber fills without ever venting. Always guard with `if (!isRunning())`.

### Speed control is inverted relative to intuition

The only speed control is the propulsion valve switching interval:

- **Longer** interval (800 ms) = chamber fills longer = stronger jet = **faster**
- **Shorter** interval (500 ms) = fires before filling = weaker jet = **slower**

To slow down, make the interval *smaller*.

### The robot cannot brake

It can only jet backwards to move forward; there is no reverse thrust. "Stopping" means
cutting propulsion and coasting to a halt against water drag. Collision avoidance therefore
depends entirely on turning early, not on stopping quickly.

### The mouse wheel must stay disabled in the console

The console captures mouse events and ignores them. Without that, the terminal translates
wheel scrolling into arrow or page keys, which are bound to the depth slider — an accidental
scroll would send a real depth command and drive the buoyancy pump.

---

## 9. Radio link details

The HC-12 link runs at 9600 baud and is **half duplex**: while the robot transmits it cannot
receive, so downlink traffic directly costs uplink reliability. Three measures keep it usable:

- Telemetry is a single compact frame, roughly 32 bytes every 2 s, under 2 % of the airtime:
  `$T,<depth_mm>,<vz>,<az>,<front_cm>,<left_cm>,<right_cm>,<roll>,<pitch>,<yaw>,<batt_cv>,<motion>*<checksum>`
  Fields are positional integers; an empty field means that sensor is invalid. The trailing
  `*XX` is an XOR checksum over everything between `$` and `*`, following NMEA convention.
  The console discards any frame whose checksum does not match rather than plotting garbage.
- Commands carry a sequence number (`w#7`). The robot recognises a repeat and re-acknowledges
  without executing it again. This matters because `w`, `j` and `k` are toggles: a retry of a
  command that did arrive would undo the action.
- The transmitter matches the whole acknowledgement line (`[OK]<command>#<seq>`) before
  clearing the retry. Matching only `[OK]` lets an unrelated reply acknowledge a command that
  was actually lost. Timeout is 400 ms with up to 5 retries.

The full sensor panel is printed to the USB console only; it is far too large for the radio.

---

## 10. Not yet verified on hardware

1. **Depth hold (`l<number>`)** — the PID output direction was inverted relative to the
   physical effect of the buoyancy valves and has been corrected by reasoning about the
   pneumatics, but closed-loop convergence has not been tested in water. Keep a hand on `s`.
2. **Obstacle avoidance (`q`) thresholds** — 12 cm ahead, 10 cm to the sides, chosen for the
   current tank. This mode compares raw distance and does not extrapolate closing speed, and
   the robot cannot brake, so verify with supervision before leaving it unattended.
3. **Ultrasonic behaviour at very short range** — most modules return either an invalid
   reading or a spurious large value below roughly 2–4 cm. An invalid reading is handled
   safely (propulsion stops); a spurious large value is not. Confirm which one happens before
   operating close to the tank wall.

## 11. Known limitations

- **OTA rollback is not available.** `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` is a bootloader
  option, and OTA can only replace the application partition, not the bootloader. With the
  board sealed, firmware that crashes before it can bring up WiFi cannot be recovered without
  physical access. Keep the previously working binary and change one thing at a time.
- The `factory` partition still holds the last image flashed over USB and is never touched by
  OTA, so it remains a clean image — but selecting it also requires code that runs.
- TEST mode (`mt`) switches the radio off entirely, including the robot's own access point.
  OTA is unavailable until `md` returns the system to DEBUG mode.
