/**********************************************************************
 * CommandHandler.h
 *
 * ESP32 侧命令解析器：模式切换、手动运动、目标深度、调试命令。
 * 命令来源：USB 串口 / HC-12 无线（共用同一命令集）。
 *********************************************************************/

#ifndef SQUID_COMMAND_HANDLER_H
#define SQUID_COMMAND_HANDLER_H

#include <cstdint>
#include <string>

#include "AutoNavigator.h"
#include "DepthController.h"
#include "ForwardControl.h"
#include "TurnControl.h"
#include "MotionLink.h"
#include "SDLogger.h"
#include "SensorHub.h"

class StatusDisplay;

class CommandHandler {
public:
    enum ControlMode : uint8_t {
        MODE_MANUAL = 0,
        MODE_AUTO = 1
    };

    CommandHandler();

    void begin();

    void setSensorHub(SensorHub* hub);
    void setStatusDisplay(StatusDisplay* display);
    void setSDLogger(SDLogger* logger);

    void setEnterTestModeCallback(void (*cb)());
    void setEnterDebugModeCallback(void (*cb)());
    void setSystemStatusCallback(void (*cb)());

    void setMotionLink(MotionLink* motionLink);
    void setForwardControl(ForwardControl* forwardControl);
    void setLeftTurnControl(TurnControl* leftTurnControl);
    void setRightTurnControl(TurnControl* rightTurnControl);
    void setDepthController(DepthController* depthController);
    void setAutoNavigator(AutoNavigator* autoNavigator);

    void processSerialInput();
    void processHC12Input();

    void update(uint32_t nowMs);

    ControlMode getMode() const;
    bool isBalanceLocked() const { return _globalBalancing; }

    // 遥控失联保护：有动作在执行且超过 _failsafeMs 没收到任何命令 → 自动急停+平衡。
    // 控制台每 5s 发一次 hb 心跳；任何命令都会刷新计时。fs 命令可查/改/关。
    uint32_t getFailsafeMs() const { return _failsafeMs; }

private:
    std::string _cmdBuffer;
    std::string _hc12Buffer;
    // HC-12 收到最后一个字节的时刻。无线上偶尔会落单个噪声字节，它不带换行，
    // 会一直赖在 _hc12Buffer 里，把下一条真命令拼成 "<噪声>w" —— 白名单判假、
    // 静默丢弃、连 [OK] 都不回，表现就是"第一次按没反应"。超时即丢弃半截帧。
    uint32_t    _hc12LastByteMs = 0;
    // 桥给每条命令编号（cmd#n）。重发用同一个号，机器人认出重号就只回执、
    // 不重复执行 —— w/j/k 都是切换式命令，重发一次等于把刚起来的动作关掉。
    // 0xFF = 还没收到过任何带号的命令（开机/重启后第一条一定执行）。
    uint8_t     _lastHc12Seq = 0xFF;

    SDLogger*     _sdLogger;
    SensorHub*    _sensorHub;
    StatusDisplay* _statusDisplay;
    MotionLink*   _motionLink;
    ForwardControl* _forwardControl;
    TurnControl* _leftTurnControl;
    TurnControl* _rightTurnControl;
    DepthController* _depthController;
    AutoNavigator* _autoNavigator;

    ControlMode _mode;
    bool        _globalBalancing;
    uint32_t    _globalBalanceEndMs;

    void (*_enterTestModeCb)();
    void (*_enterDebugModeCb)();
    void (*_sysStatusCb)();

    uint32_t _lastCmdMs   = 0;
    uint32_t _failsafeMs  = 20000;   // 0 = 关闭
    bool     _failsafeFired = false;

    void processCommand(const std::string& cmd, bool fromHC12 = false);
    void checkFailsafe(uint32_t nowMs);
    void toggleMode();
    void enterManualMode();
    void enterAutoMode();
    void handleManualCommand(const std::string& cmd);
    void handleDepthTargetCommand(const std::string& cmd, bool fromHC12 = false);
    void handleHC12ChannelCommand(const std::string& channel);
    void handleCalibrateCommand();
    void handleStopCommand();
    void printModeBanner() const;
};

#endif  // SQUID_COMMAND_HANDLER_H
