/**********************************************************************
 * CommandHandler.cpp
 *
 * ESP32 侧命令解析和控制模式切换。并发规则：
 * 1. 前进/转向/浮沉属不同子系统，可同时执行。
 * 2. 左转和右转同一子系统，后到命令覆盖前一条方向。
 * 3. 手动浮沉清除定深目标，但不打断前进或转向。
 *********************************************************************/

#include "CommandHandler.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

#include "Board.h"
#include "Calibration.h"
#include "Console.h"
#include "MotionLock.h"
#include "Output.h"
#include "StatusDisplay.h"
#include "Sys.h"
#include "driver/gpio.h"
#include "driver/uart.h"

namespace {
void trim(std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) { s.clear(); return; }
    size_t b = s.find_last_not_of(" \t\r\n");
    s = s.substr(a, b - a + 1);
}

std::string normalizeCommand(std::string cmd) {
    trim(cmd);
    for (char& c : cmd) c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
    return cmd;
}

bool startsWith(const std::string& s, const char* prefix) {
    return s.rfind(prefix, 0) == 0;
}

bool isDigitCh(char c) { return isdigit(static_cast<unsigned char>(c)) != 0; }

// HC-12 帧间超时：超过这么久没有新字节，就把没收完的半截帧丢掉。
constexpr uint32_t HC12_FRAME_GAP_MS = 200;

// HC-12 白名单：只有匹配已知模式的命令才允许执行，过滤噪声。
bool isValidHC12Command(const std::string& cmd) {
    if (cmd.empty()) return false;

    static const char singleCmds[] = {'w', 'a', 'd', 's', 'j', 'k', 'q', 'g', 'c', 'h'};
    if (cmd.length() == 1) {
        for (char c : singleCmds) {
            if (cmd[0] == c) return true;
        }
        return false;
    }

    if (cmd == "mt" || cmd == "md" || cmd == "hb" || cmd == "fs") return true;
    if (cmd == "pget" || cmd == "psave" || startsWith(cmd, "pset ")) return true;
    if (startsWith(cmd, "fs") && cmd.length() > 2 && isDigitCh(cmd[2])) return true;

    if (cmd[0] == 'l' && cmd.length() > 1) {
        for (size_t i = 1; i < cmd.length(); i++) {
            if (!isDigitCh(cmd[i]) && cmd[i] != '.') return false;
        }
        return true;
    }

    if (startsWith(cmd, "mark")) return true;

    if (cmd.length() == 6 && startsWith(cmd, "esp") &&
        isDigitCh(cmd[3]) && isDigitCh(cmd[4]) && isDigitCh(cmd[5])) return true;

    if (cmd == "forward" || cmd == "left"   || cmd == "right" ||
        cmd == "stop"    || cmd == "ascend"  || cmd == "descend" ||
        cmd == "sensors" || cmd == "calibrate" ||
        cmd == "help") return true;

    return false;
}

void printHelp() {
    g_dbg->println("\r\n==================== 命令帮助 (h) ====================");
    g_dbg->println("[运行模式] 仅手动切换 | 默认: DEBUG");
    g_dbg->println("  md            DEBUG: WiFi开/OTA/文件管理网页, 默认打印传感器");
    g_dbg->println("  mt            TEST : WiFi关, SD 记录 20Hz 训练日志");
    g_dbg->println("  stat          打印 SD 记录统计(文件/大小)");
    g_dbg->println("  sd            SD 卡挂载诊断(多组参数逐一尝试)");
    g_dbg->println("  ps            系统状态(任务栈余量/堆/心跳/SD/WiFi)");
    g_dbg->println("  fs / fs<秒>   遥控失联保护: 查看/设置(默认20s, fs0 关闭)");
    g_dbg->println("");
    g_dbg->println("[执行器时序参数] 存在 Minima 的 EEPROM, 断电保留");
    g_dbg->println("  pget          回读全部参数(编号 名字 值)");
    g_dbg->println("  pset <名字|编号> <值>   立即生效, 例 pset fwd_interval 700");
    g_dbg->println("  psave         保存当前参数到执行器 EEPROM");
    g_dbg->println("");
    g_dbg->println("[控制模式] | 默认: MANUAL");
    g_dbg->println("  q             切换 手动 <-> 自动(自动避障)");
    g_dbg->println("");
    g_dbg->println("[手动运动] 仅 MANUAL 模式可用(自动模式下拒绝)");
    g_dbg->println("  w / forward   前进 开/关(再按停)");
    g_dbg->println("  a / left      左转一次序列");
    g_dbg->println("  d / right     右转一次序列");
    g_dbg->println("  j / descend   下沉 开/关(吸气)");
    g_dbg->println("  k / ascend    上浮 开/关(排气)");
    g_dbg->println("  l<数字>       定深到 N cm (如 l30, 需深度在线)");
    g_dbg->println("  s / stop      急停(始终可用; 之后锁定5s气压平衡)");
    g_dbg->println("");
    g_dbg->println("[传感器/显示/日志]");
    g_dbg->println("  g / sensors   全部传感器显示 开/收起 (默认开)");
    g_dbg->println("  c / calibrate 当前深度标定为零点");
    g_dbg->println("  mark <备注>   在 events.log 插入时间戳标记");
    g_dbg->println("");
    g_dbg->println("[HC-12 无线信道] 同步顺序: 先 ESP### 再 HC###");
    g_dbg->println("  ESP025        设本机(机器人)HC-12 -> 信道 025");
    g_dbg->println("  HC025         (经 Bridge 发) 设桥端信道, Bridge 拦截");
    g_dbg->println("");
    g_dbg->println("  h / help      显示本帮助");
    g_dbg->println("说明: 收起传感器后, 仅保留 命令回应 + Minima回传(<- Minima:)");
    g_dbg->println("       命令来源: USB串口 / HC-12无线 (共用)");
    g_dbg->println("====================================================\r\n");
}
}  // namespace

CommandHandler::CommandHandler()
    : _cmdBuffer(""),
      _hc12Buffer(""),
      _sdLogger(nullptr),
      _sensorHub(nullptr),
      _statusDisplay(nullptr),
      _motionLink(nullptr),
      _forwardControl(nullptr),
      _leftTurnControl(nullptr),
      _rightTurnControl(nullptr),
      _depthController(nullptr),
      _autoNavigator(nullptr),
      _mode(MODE_MANUAL),
      _globalBalancing(false),
      _globalBalanceEndMs(0),
      _enterTestModeCb(nullptr),
      _enterDebugModeCb(nullptr) {}

void CommandHandler::begin() {
    _cmdBuffer         = "";
    _hc12Buffer        = "";
    _mode              = MODE_MANUAL;
    _globalBalancing   = false;
    _globalBalanceEndMs = 0;
    _lastCmdMs          = millis();

    // HC-12 SET 脚：输出，默认高（透传模式）。
    gpio_config_t setCfg = {};
    setCfg.pin_bit_mask = 1ULL << HC12_SET_PIN;
    setCfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&setCfg);
    gpio_set_level(HC12_SET_PIN, 1);

    // HC-12 UART2。
    const uart_config_t cfg = {
        .baud_rate  = HC12_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // TX 缓冲放大到 4KB：9600 波特下调试文字发得慢，缓冲满时 uart_write_bytes
    // 会阻塞主循环（实测每秒卡 ~200ms，打乱前进阀门节拍）。Hc12Output 另做满则丢弃。
    uart_driver_install(HC12_UART_PORT, 512, HC12_TX_BUF_SIZE, 0, nullptr, 0);
    uart_param_config(HC12_UART_PORT, &cfg);
    uart_set_pin(HC12_UART_PORT, HC12_TX_PIN, HC12_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    g_dbg->println("[CommandHandler] HC-12 serial initialized.");
}

void CommandHandler::setSDLogger(SDLogger* logger) { _sdLogger = logger; }
void CommandHandler::setEnterTestModeCallback(void (*cb)())  { _enterTestModeCb  = cb; }
void CommandHandler::setEnterDebugModeCallback(void (*cb)()) { _enterDebugModeCb = cb; }
void CommandHandler::setSystemStatusCallback(void (*cb)())  { _sysStatusCb = cb; }
void CommandHandler::setSensorHub(SensorHub* hub) { _sensorHub = hub; }
void CommandHandler::setStatusDisplay(StatusDisplay* display) { _statusDisplay = display; }
void CommandHandler::setMotionLink(MotionLink* motionLink) { _motionLink = motionLink; }
void CommandHandler::setForwardControl(ForwardControl* forwardControl) { _forwardControl = forwardControl; }
void CommandHandler::setLeftTurnControl(TurnControl* leftTurnControl) { _leftTurnControl = leftTurnControl; }
void CommandHandler::setRightTurnControl(TurnControl* rightTurnControl) { _rightTurnControl = rightTurnControl; }
void CommandHandler::setDepthController(DepthController* depthController) { _depthController = depthController; }
void CommandHandler::setAutoNavigator(AutoNavigator* autoNavigator) { _autoNavigator = autoNavigator; }

void CommandHandler::processSerialInput() {
    int ch;
    while ((ch = consoleReadByte()) >= 0) {
        const char c = static_cast<char>(ch);

        if (c == '\n' || c == '\r') {
            if (!_cmdBuffer.empty()) {
                const std::string cmd = normalizeCommand(_cmdBuffer);
                if (_sdLogger) _sdLogger->logCommand(millis(), "serial", cmd.c_str());
                processCommand(cmd);
            }
            _cmdBuffer.clear();
            continue;
        }

        _cmdBuffer += c;
        if (_cmdBuffer.length() > 128) _cmdBuffer.clear();
    }
}

void CommandHandler::processHC12Input() {
    const uint32_t nowMs = millis();

    // 帧间超时：一条命令最长十几个字符，9600 波特下不到 20ms 就收全了。
    // 缓冲里有东西却 HC12_FRAME_GAP_MS 没续上 → 那是噪声或半截帧，丢掉，
    // 否则它会粘在下一条真命令前面把整条吃掉（见 CommandHandler.h 的说明）。
    if (!_hc12Buffer.empty() && nowMs - _hc12LastByteMs >= HC12_FRAME_GAP_MS) {
        _hc12Buffer.clear();
    }

    uint8_t b = 0;
    while (uart_read_bytes(HC12_UART_PORT, &b, 1, 0) == 1) {
        _hc12LastByteMs = nowMs;
        const char c = static_cast<char>(b);

        if (c == '\n' || c == '\r') {
            if (!_hc12Buffer.empty()) {
                const std::string raw = normalizeCommand(_hc12Buffer);

                // 桥发过来的是 "<命令>#<序号>"。拆出序号，命令本身按原样处理。
                std::string cmd = raw;
                int seq = -1;
                const size_t hash = raw.rfind('#');
                if (hash != std::string::npos && hash + 1 < raw.size()) {
                    bool allDigits = true;
                    for (size_t k = hash + 1; k < raw.size(); ++k) {
                        if (!isDigitCh(raw[k])) { allDigits = false; break; }
                    }
                    if (allDigits) {
                        seq = atoi(raw.c_str() + hash + 1);
                        cmd = raw.substr(0, hash);
                    }
                }

                if (!isValidHC12Command(cmd)) {
                    _hc12Buffer.clear();
                    continue;
                }

                // 重号 = 桥没收到上一次的回执在重发。命令早就执行过了，这里只补回执。
                // 少了这一步，重发会把 w/j/k 这类切换式命令反向执行一次。
                const bool duplicate = (seq >= 0 && static_cast<uint8_t>(seq) == _lastHc12Seq);
                if (!duplicate) {
                    if (seq >= 0) _lastHc12Seq = static_cast<uint8_t>(seq);
                    // 必须在 processCommand 之前 log，否则 "md" 会先 endSession 再 log。
                    if (_sdLogger) _sdLogger->logCommand(millis(), "hc12", cmd.c_str());
                    processCommand(cmd, /*fromHC12=*/true);
                }

                const bool isChannelCmd =
                    cmd.length() == 6 && startsWith(cmd, "esp") &&
                    isDigitCh(cmd[3]) && isDigitCh(cmd[4]) && isDigitCh(cmd[5]);
                if (!isChannelCmd) {
                    // 回执必须原样带回序号，桥才能认出是哪一条的回执。
                    // 一次写完：Hc12Output 装不下会整段丢弃，拆成多次 print 会撕成半行。
                    const std::string ack = "[OK]" + raw + "\r\n";
                    g_hc12Out.print(ack);
                }
            }
            _hc12Buffer.clear();
            continue;
        }

        _hc12Buffer += c;
        if (_hc12Buffer.length() > 64) {
            _hc12Buffer.clear();
        }
    }
}

CommandHandler::ControlMode CommandHandler::getMode() const {
    return _mode;
}

void CommandHandler::update(uint32_t nowMs) {
    if (_globalBalancing && nowMs >= _globalBalanceEndMs) {
        _globalBalancing = false;
        g_dbg->println("Global balance complete. System ready.");
    }
    checkFailsafe(nowMs);
}

// 遥控失联保护：只在"有东西在动"时才触发；停着不动时链路断了不必急停。
void CommandHandler::checkFailsafe(uint32_t nowMs) {
    if (_failsafeMs == 0 || _globalBalancing || _lastCmdMs == 0) return;

    const bool moving =
        (_forwardControl   && _forwardControl->isRunning()) ||
        (_leftTurnControl  && _leftTurnControl->isRunning()) ||
        (_rightTurnControl && _rightTurnControl->isRunning()) ||
        (_depthController  && (_depthController->getManualDirection() != BUOYANCY_STOP ||
                               _depthController->isHoldingTarget()));
    if (!moving) {
        _failsafeFired = false;
        return;
    }
    if (nowMs - _lastCmdMs < _failsafeMs) {
        _failsafeFired = false;
        return;
    }
    if (_failsafeFired) return;
    _failsafeFired = true;
    g_dbg->print("[FAILSAFE] 已 ");
    g_dbg->print(static_cast<int>((nowMs - _lastCmdMs) / 1000));
    g_dbg->println(" s 没收到任何命令 —— 遥控可能失联，自动急停并气压平衡。");
    if (_sdLogger) _sdLogger->logEvent(nowMs, "failsafe: command link lost");
    MotionLockGuard motionGuard;
    handleStopCommand();
}

void CommandHandler::processCommand(const std::string& cmd, bool fromHC12) {
    if (cmd.empty()) {
        return;
    }
    _lastCmdMs = millis();     // 任何命令都刷新失联计时
    _failsafeFired = false;

    if (cmd == "hb") {         // 控制台心跳：只刷新计时，不做别的（HC-12 侧会回 [OK]hb）
        return;
    }

    if (cmd == "fs" || (startsWith(cmd, "fs") && cmd.length() > 2 && isDigitCh(cmd[2]))) {
        if (cmd.length() > 2) {
            const long secs = strtol(cmd.c_str() + 2, nullptr, 10);
            _failsafeMs = (secs <= 0) ? 0 : static_cast<uint32_t>(secs) * 1000U;
        }
        if (_failsafeMs == 0) {
            g_dbg->println("[FAILSAFE] 已关闭（fs<秒> 开启，如 fs20）");
        } else {
            g_dbg->print("[FAILSAFE] 失联 ");
            g_dbg->print(static_cast<int>(_failsafeMs / 1000));
            g_dbg->println(" s 自动急停（fs0 关闭）");
        }
        return;
    }

    if (cmd == "mt") {
        if (_enterTestModeCb) _enterTestModeCb();
        return;
    }

    if (cmd == "md") {
        if (_enterDebugModeCb) _enterDebugModeCb();
        return;
    }

    if (cmd == "stat") {
        if (_sdLogger) _sdLogger->printStats();
        return;
    }

    if (cmd == "ps") {   // 任务栈 / 堆 / 各子系统状态
        if (_sysStatusCb) _sysStatusCb();
        return;
    }

    // ── 执行器时序参数（存在 Minima 的 EEPROM 里）──
    if (cmd == "pget") {
        if (_motionLink) _motionLink->requestParams();
        return;
    }
    if (cmd == "psave") {
        if (_motionLink) _motionLink->saveParams();
        g_dbg->println("[PARAM] 已请求执行器保存到 EEPROM");
        return;
    }
    if (startsWith(cmd, "pset")) {
        // 用法：pset <编号|名字> <值>
        std::string rest = cmd.length() > 4 ? cmd.substr(4) : "";
        trim(rest);
        const size_t sp = rest.find(' ');
        if (sp == std::string::npos) {
            g_dbg->println("[PARAM] 用法: pset <编号|名字> <值>   (pget 查看列表)");
            return;
        }
        const std::string key = rest.substr(0, sp);
        const long value = strtol(rest.c_str() + sp + 1, nullptr, 10);
        int id = -1;
        if (isDigitCh(key[0])) {
            id = static_cast<int>(strtol(key.c_str(), nullptr, 10));
        } else {
            for (uint8_t i = 0; i < P_COUNT; i++) {
                if (key == PARAM_NAMES[i]) { id = i; break; }
            }
        }
        if (id < 0 || id >= P_COUNT || value < 0 || value > 65535) {
            g_dbg->println("[PARAM] 参数名/编号或数值无效");
            return;
        }
        if (_motionLink) _motionLink->setParam(static_cast<uint8_t>(id), static_cast<uint16_t>(value));
        // 转向时长本地也要跟着改，保证界面状态与执行器一致。
        if (id == P_TURN_DURATION) {
            if (_leftTurnControl)  _leftTurnControl->setDurationMs(static_cast<uint16_t>(value));
            if (_rightTurnControl) _rightTurnControl->setDurationMs(static_cast<uint16_t>(value));
        }
        return;
    }

    if (cmd == "sd") {   // SD 挂载诊断（耗时数秒，不持运动锁）
        if (_sdLogger) _sdLogger->diagnose();
        return;
    }

    if (startsWith(cmd, "mark")) {
        const std::string note = (cmd.length() > 5) ? cmd.substr(5) : std::string("(no note)");
        if (_sdLogger) _sdLogger->logEvent(millis(), note.c_str());
        g_dbg->print("Mark logged: ");
        g_dbg->println(note);
        return;
    }

    if (_globalBalancing && cmd != "s" && cmd != "stop") {
        g_dbg->println("Commands locked: pressure balance in progress.");
        return;
    }

    if (cmd.length() == 6 && startsWith(cmd, "esp") &&
        isDigitCh(cmd[3]) && isDigitCh(cmd[4]) && isDigitCh(cmd[5])) {
        handleHC12ChannelCommand(cmd.substr(3));
        return;
    }

    // 以下命令会改运动控制对象：持锁执行（都是瞬时操作）。上面的 mt/md/esp###
    // 等慢操作（WiFi 重连、HC-12 AT 配置）不持锁，运动任务照常按时推进。
    MotionLockGuard motionGuard;

    if (cmd == "q") {
        toggleMode();
        return;
    }

    if (cmd == "h" || cmd == "help") {
        if (fromHC12) {
            // 整份帮助 1800+ 字节 = 9600 波特下链路被独占近 2 秒，期间 HC-12
            // 半双工收不到任何命令。无线上只回一行速查，完整版留给 USB。
            Output* saved = g_dbg;
            g_dbg = g_usb;
            printHelp();
            g_dbg = saved;
            // 一次 print 发完（含换行）：Hc12Output 装不下会整段丢弃，
            // 分成 print+println 两次可能只发出半行。
            g_hc12Out.print("[HELP] w前进 a左 d右 j上浮 k下沉 s急停 l<cm>定深 "
                            "q自动 g面板 c标零 mt/md模式 fs失联 (完整帮助见USB)\r\n");
        } else {
            printHelp();
        }
        return;
    }

    if (cmd == "g" || cmd == "sensors") {
        if (_sensorHub) {
            const bool on = _sensorHub->toggleDisplay();
            g_dbg->print("全部传感器显示: ");
            g_dbg->println(on ? "开" : "收起");
            if (on) _sensorHub->forceDisplayAll();
        }
        return;
    }

    if (cmd == "c" || cmd == "calibrate") {
        handleCalibrateCommand();
        return;
    }

    if (cmd == "s" || cmd == "stop") {
        handleStopCommand();
        return;
    }

    if (startsWith(cmd, "l") && cmd.length() > 1) {
        handleDepthTargetCommand(cmd, fromHC12);
        return;
    }

    if (_mode == MODE_AUTO) {
        g_dbg->println("AUTO mode is active. Press q to return to manual mode.");
        return;
    }

    handleManualCommand(cmd);
}

void CommandHandler::toggleMode() {
    if (_mode == MODE_MANUAL) {
        enterAutoMode();
    } else {
        enterManualMode();
    }
}

void CommandHandler::enterManualMode() {
    _mode = MODE_MANUAL;

    if (_autoNavigator) _autoNavigator->setEnabled(false);
    if (_forwardControl) _forwardControl->emergencyStop();
    if (_leftTurnControl) _leftTurnControl->cancel();
    if (_rightTurnControl) _rightTurnControl->cancel();
    if (_depthController) {
        _depthController->clearTarget();
        _depthController->manualStop();
    }

    printModeBanner();
}

void CommandHandler::enterAutoMode() {
    _mode = MODE_AUTO;

    if (_forwardControl) _forwardControl->emergencyStop();
    if (_leftTurnControl) _leftTurnControl->cancel();
    if (_rightTurnControl) _rightTurnControl->cancel();

    if (_depthController) {
        _depthController->manualStop();
        if (_sensorHub && _sensorHub->isDepthOnline()) {
            _depthController->holdCurrentDepth();
        } else {
            _depthController->clearTarget();
        }
    }

    if (_autoNavigator) _autoNavigator->setEnabled(true);

    printModeBanner();
}

void CommandHandler::handleManualCommand(const std::string& cmd) {
    if (!_forwardControl || !_leftTurnControl || !_rightTurnControl || !_depthController) {
        g_dbg->println("Motion modules are not ready.");
        return;
    }

    if (cmd == "w" || cmd == "forward") {
        // 平衡窗口由执行器本地掌握；这里只按意图切换。
        if (_forwardControl->isRunning()) {
            _forwardControl->stop();
            g_dbg->println("Forward stop: pressure balance in progress...");
        } else {
            _forwardControl->start();
            g_dbg->println("Forward started.");
        }
        return;
    }

    // 左右转向、上浮下沉可能接反，映射集中在 Calibration.h，这里只按逻辑方向写。
    if (cmd == "a" || cmd == "left" || cmd == "d" || cmd == "right") {
        const bool wantLeft = (cmd == "a" || cmd == "left");
        TurnControl* want = (wantLeft == cal::turnLeftIsLeft()) ? _leftTurnControl
                                                                : _rightTurnControl;
        TurnControl* other = (want == _leftTurnControl) ? _rightTurnControl : _leftTurnControl;
        other->cancel();
        want->start();
        return;
    }

    // 实测：j 是吸气 → 下沉，k 是排气 → 上浮（见 Calibration.h）。
    // 按键对应的物理动作没有改，只是把名字摆正了。
    if (cmd == "j" || cmd == "descend" || cmd == "k" || cmd == "ascend") {
        const bool wantSink = (cmd == "j" || cmd == "descend");
        if (wantSink) {
            _depthController->manualDescend();
        } else {
            _depthController->manualAscend();
        }
        const bool stopped = _depthController->getManualDirection() == BUOYANCY_STOP;
        g_dbg->println(stopped ? (wantSink ? "Descend stopped." : "Ascend stopped.")
                               : (wantSink ? "Descending..." : "Ascending..."));
        return;
    }

    g_dbg->print("Unknown manual command: ");
    g_dbg->println(cmd);
}

void CommandHandler::handleDepthTargetCommand(const std::string& cmd, bool fromHC12) {
    // g_dbg 就是 USB + HC-12 两路（main.cpp 里 g_dbg = &g_tee，之后不再改），
    // 所以这里写一次就够。原来还额外往 g_hc12Out 补发一份 —— 同一句话在 9600
    // 的半双工链路上发两遍，纯浪费空口时间。
    (void)fromHC12;
    auto reply = [&](const char* msg) { g_dbg->println(msg); };

    if (!_depthController) {
        reply("[ERR] Depth controller not ready.");
        return;
    }

    const float targetDepthCm = strtof(cmd.substr(1).c_str(), nullptr);

    // l0 = 刻度 0：清掉定深目标并持续上浮，直到操作员另下命令（控制台滑块最上一档）。
    // 上浮是安全动作，深度传感器离线也允许（定深才需要深度在线）。
    if (targetDepthCm == 0.0f) {
        _depthController->clearTarget();
        if (_depthController->getManualDirection() != cal::BUOY_RISE) {
            _depthController->manualAscend();   // 真·上浮（见 Calibration.h）
        }
        reply("Surfacing: ascend until next command.");
        return;
    }

    if (targetDepthCm < 0.0f) {
        reply("[ERR] Depth target must be >= 0 cm.");
        return;
    }

    if (_sensorHub && !_sensorHub->isDepthOnline()) {
        reply("[ERR] Depth sensor offline, cannot set target.");
        return;
    }

    _depthController->setTargetDepth(targetDepthCm);
    // 这一行本身就会经 g_dbg 上无线，控制台按 "Target depth set to " 解析出目标深度；
    // processHC12Input() 之后还会补一条 "[OK]l<n>" 回执。原来这里再单独发一份
    // "[OK] depth target=..." 是第三遍，只会占空口时间。
    g_dbg->print("Target depth set to ");
    g_dbg->print(targetDepthCm, 1);
    g_dbg->println(" cm");
}

void CommandHandler::handleCalibrateCommand() {
    if (_autoNavigator) _autoNavigator->setEnabled(false);
    if (_forwardControl) _forwardControl->emergencyStop();
    if (_leftTurnControl) _leftTurnControl->cancel();
    if (_rightTurnControl) _rightTurnControl->cancel();

    if (_depthController) {
        _depthController->clearTarget();
        _depthController->manualStop();
        _depthController->resetAfterCalibration();
    }

    if (_sensorHub) _sensorHub->calibrateDepthZero();

    g_dbg->println("Depth zero recalibrated.");

    if (_mode == MODE_AUTO && _depthController && _autoNavigator) {
        if (_sensorHub && _sensorHub->isDepthOnline()) {
            _depthController->holdCurrentDepth();
        } else {
            _depthController->clearTarget();
        }
        _autoNavigator->setEnabled(true);
    }
}

void CommandHandler::handleStopCommand() {
    _mode = MODE_MANUAL;

    if (_autoNavigator) _autoNavigator->setEnabled(false);

    if (_forwardControl) _forwardControl->emergencyStop();
    if (_leftTurnControl) _leftTurnControl->cancel();
    if (_rightTurnControl) _rightTurnControl->cancel();
    if (_depthController) {
        _depthController->clearTarget();
        _depthController->manualStop();
    }
    if (_motionLink) _motionLink->emergencyStop();

    // 原有 delay(10) 用于隔开急停帧与平衡帧；现在平衡帧由 motion 任务下一拍(≤5ms)
    // 发出，已天然隔开。持 MotionLock 时不能 delay，否则阻塞运动任务。
    if (_forwardControl)   _forwardControl->forceBalance();
    if (_leftTurnControl)  _leftTurnControl->forceBalance();
    if (_rightTurnControl) _rightTurnControl->forceBalance();
    if (_depthController)  _depthController->forceBalance();

    _globalBalancing    = true;
    _globalBalanceEndMs = millis() + 5000;

    g_dbg->println("EMERGENCY STOP: global pressure balance started (5s), commands locked.");
}

void CommandHandler::handleHC12ChannelCommand(const std::string& channel) {
    g_dbg->print("[HC-12] Setting channel to ");
    g_dbg->print(channel);
    g_dbg->print(" ...");

    // 先在当前信道通知 Minima。
    g_hc12Out.print("[Robot] Switching to ch.");
    g_hc12Out.print(channel);
    g_hc12Out.print(", send HC");
    g_hc12Out.print(channel);
    g_hc12Out.println(" now");
    delay(100);

    // 拉低 SET 脚，进入 AT 命令模式。
    gpio_set_level(HC12_SET_PIN, 0);
    delay(200);

    g_hc12Out.print("AT+C");
    g_hc12Out.println(channel);

    // 等待 HC-12 响应，最多 1000ms。
    std::string response;
    const uint32_t start = millis();
    while (millis() - start < 1000) {
        uint8_t c = 0;
        if (uart_read_bytes(HC12_UART_PORT, &c, 1, 0) == 1) {
            response += static_cast<char>(c);
        }
    }

    // 拉高 SET 脚，退出 AT 模式。
    gpio_set_level(HC12_SET_PIN, 1);
    delay(200);

    trim(response);
    if (response.find("OK") != std::string::npos) {
        g_dbg->print(" OK (ch.");
        g_dbg->print(channel);
        g_dbg->print(") — now send HC");
        g_dbg->print(channel);
        g_dbg->println(" to bridge end");
    } else {
        g_dbg->print(" FAILED");
        if (!response.empty()) {
            g_dbg->print(": ");
            g_dbg->print(response);
        } else {
            g_dbg->print(": no response — check SET wiring (IO47)");
        }
        g_dbg->println();
    }
}

void CommandHandler::printModeBanner() const {
    g_dbg->println("---------------------------------------------");
    g_dbg->print("Control mode: ");
    g_dbg->println(_mode == MODE_AUTO ? "AUTO" : "MANUAL");
    g_dbg->println("---------------------------------------------");
}
