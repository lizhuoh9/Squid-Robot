/**********************************************************************
 * main.cpp — Squid-Robot 主控（原生 ESP-IDF，从 ESP32.ino 迁移）
 *
 * 职责：读取深度/超声波/IMU；卡尔曼滤波；手动/自动模式；计算前进、
 * 转向、浮沉三子系统执行器掩码并发给 Minima；SD 20Hz 训练日志；
 * USB 串口 + HC-12 无线控制；WiFi/OTA/SD 文件管理网页。
 *
 * Arduino 的 setup()/loop() 在这里由 app_main() 里的初始化 + 主循环替代。
 *
 * 任务划分（2026-09-19）：
 *   motion 任务（优先级 10，CPU1，每 5ms 严格定时）：定深 PID、前进/转向/浮沉
 *     时序、执行器掩码下发 Minima。与打印/SD/WiFi 等完全隔离，阀门节拍不受其影响。
 *   主循环（app_main，优先级 1，CPU0）：传感器读取、命令解析、自动导航决策、
 *     SD 记录、状态打印、WiFi/OTA。深度数据以快照形式交给 motion 任务。
 *   两边共享的运动控制对象由 MotionLock 保护。
 *********************************************************************/

#include <cstdint>
#include <functional>
#include <string>

#include "AutoNavigator.h"
#include "Board.h"
#include "CH9434A.h"
#include "CommandHandler.h"
#include "Console.h"
#include "Calibration.h"
#include "DepthController.h"
#include "DepthSensorManager.h"
#include "ForwardControl.h"
#include "ImuManager.h"
#include "MotionLink.h"
#include "MotionLock.h"
#include "Output.h"
#include "OtaManager.h"
#include "TurnControl.h"
#include "SDLogger.h"
#include "SensorHub.h"
#include "StatusDisplay.h"
#include "Sys.h"
#include "UltrasonicManager.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_wifi.h"

// ── 全局对象 ────────────────────────────────────────────────────────
static CH9434A            ch9434(SPI_CS, CH9434A_INT);
static DepthSensorManager depthMgr;
static SDLogger           sdLogger;
static UltrasonicManager  ultrasonicMgr(&ch9434);
static ImuManager         imuMgr(&ch9434);
static SensorHub          sensorHub;

static MotionLink         motionLink;
static ForwardControl     forwardControl;
static TurnControl        leftTurnControl;
static TurnControl        rightTurnControl;
static DepthController    depthController;
static AutoNavigator      autoNavigator;
static OtaManager         otaManager;

static CommandHandler     cmdHandler;
static StatusDisplay      statusDisplay;

namespace {
constexpr uint32_t STARTUP_DEPTH_CALIBRATION_DELAY_MS = 2000;
constexpr uint8_t  DEPTH_INIT_RETRIES                  = 3;
constexpr uint32_t DEPTH_INIT_RETRY_DELAY_MS           = 200;
constexpr uint32_t SD_LOG_INTERVAL_MS                  = 50;   // 20 Hz
constexpr uint32_t CONTROL_TELEMETRY_INTERVAL_MS      = 500;  // PC 별도 CSV용

bool     gStartupDepthCalibrationDone = false;
uint32_t gLastLogMs                   = 0;
uint32_t gLastControlTelemetryMs      = 0;
bool     gWebServerStarted            = false;

// 主循环 → motion 任务的深度快照（持 MotionLock 读写）。stamp 变化 = 有新采样。
struct DepthSnapshot {
    bool     valid   = false;
    uint32_t stamp   = 0;
    float    depthCm = 0.0f;
    float    speedCmS = 0.0f;
    float    accelCmS2 = 0.0f;
};
DepthSnapshot gDepthSnap;
TaskHandle_t  gMotionTask = nullptr;

constexpr uint32_t    MOTION_PERIOD_MS   = 5;    // 运动控制周期
constexpr UBaseType_t MOTION_TASK_PRIO   = 10;   // 高于主循环(1)，低于 WiFi(23)
constexpr BaseType_t  MOTION_TASK_CORE   = 1;    // WiFi/主循环在 CPU0
constexpr uint32_t    MOTION_TASK_STACK  = 4096;

enum RobotMode : uint8_t { ROBOT_DEBUG, ROBOT_TEST };
RobotMode gRobotMode = ROBOT_DEBUG;

}  // namespace

// ── 机器人运行模式切换（mt/md 手动）────────────────────────────────
static void enterTestMode() {
    if (gRobotMode == ROBOT_TEST) {
        g_usb->println("已在测试模式中。");
        return;
    }
    gRobotMode = ROBOT_TEST;
    // 若正处于配网热点模式，先干净拆掉热点(AP/DNS/HTTP)并复位状态，
    // 否则 _apActive 残留会让之后 md 重开热点失败。非热点模式下此调用直接返回。
    otaManager.stopProvisioningPortal();
    esp_wifi_stop();

    char folder[32];
    otaManager.getSessionFolderName(folder, sizeof(folder));
    const bool sdOk = sdLogger.startSession(folder);
    if (sdOk) {
        sdLogger.logEvent(millis(), "TEST mode started");
        g_usb->println(">>> 测试模式：WiFi 已关闭，SD 记录已开始。");
    } else {
        g_usb->println(">>> 测试模式：WiFi 已关闭，但 SD 记录启动失败！");
        g_usb->println("    检查 SD 卡是否插好/格式化，本次 session 不会有数据。");
    }
    g_usb->println("    WiFi 关闭期间 OTA/网页 不可用。输入 'md' 返回调试模式。");
}

static void enterDebugMode() {
    if (gRobotMode == ROBOT_DEBUG) {
        g_dbg->println("已在调试模式中。");
        return;
    }
    const uint32_t ms = millis();
    sdLogger.logEvent(ms, "TEST mode ended");
    sdLogger.endSession(ms);
    gRobotMode = ROBOT_DEBUG;

    g_usb->println(">>> 调试模式：正在重新启用 WiFi...");
    otaManager.begin(false);

    if (!otaManager.isProvisioning()) {
        otaManager.beginWebServer();
        gWebServerStarted = true;
        g_usb->println("WiFi、文件管理网页和 OTA 已恢复。");
    }
}

// 固件版本。改动行为时记得同步 —— 这是开机唯一能认出烧的是哪一版的地方。
#define SQUID_FW_VERSION "Squid-ESP32 Proposal-A EXPERIMENTAL (2026-10-08)"

static void printWelcome() {
    g_usb->println("\r\nESP32-S3 Main Controller");
    g_usb->println(SQUID_FW_VERSION);
    g_usb->println("---------------------------------------------");
    g_usb->println("Sensor topology:");
    g_usb->println("  MS5837        -> ESP32 I2C (SDA=IO5, SCL=IO4, 400kHz)");
    g_usb->println("  CH9434A UART1 -> Front ultrasonic");
    g_usb->println("  CH9434A UART2 -> Left ultrasonic");
    g_usb->println("  CH9434A UART0 -> Right ultrasonic");
    g_usb->println("  CH9434A UART3 -> IMU (EBIMU-9DOFV6, 115200)");
    g_usb->println("Control topology:");
    g_usb->println("  ESP32         -> Timing / Kalman / Auto nav");
    g_usb->println("  Minima        -> Pure actuator executor");
    g_usb->println("---------------------------------------------");
}

static void handleStartupDepthCalibration(uint32_t nowMs) {
    if (gStartupDepthCalibrationDone || nowMs < STARTUP_DEPTH_CALIBRATION_DELAY_MS) {
        return;
    }
    if (!depthMgr.isValid()) {
        return;
    }
    sensorHub.calibrateDepthZero();
    depthController.resetAfterCalibration();
    gStartupDepthCalibrationDone = true;
    g_dbg->println("Startup depth auto-calibration complete.");
}

static void motionTask(void*);   // 定义在诊断代码之后（要用到 diag::）

// 控制台 "ps"：任务栈余量 / 堆 / 运行时间。下水前确认余量，现场排查也用得上。
// 前进意图核对用的状态（函数体在下面的 syncForwardIntent()；
// 计数器要在 printSystemStatus() 的 ps 输出里用，所以声明放在这里）。
namespace {
constexpr uint32_t FWD_CHECK_GRACE_MS    = 2500;   // 意图变化后多久才开始核对
constexpr uint32_t FWD_CHECK_INTERVAL_MS = 1000;   // 两次纠正之间的最小间隔
bool     gFwdIntentPrev    = false;
uint32_t gFwdIntentChangeMs = 0;
uint32_t gFwdLastFixMs     = 0;
uint32_t gFwdFixCount      = 0;
}

void printSystemStatus() {
    const uint32_t up = millis() / 1000;
    g_dbg->printf("[SYS] 运行 %um%02us | 空闲堆 %u KB (最低 %u KB) | 最大连续块 %u KB\r\n",
                  static_cast<unsigned>(up / 60), static_cast<unsigned>(up % 60),
                  static_cast<unsigned>(esp_get_free_heap_size() / 1024),
                  static_cast<unsigned>(esp_get_minimum_free_heap_size() / 1024),
                  static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT) / 1024));
    if (gMotionTask) {
        g_dbg->printf("[SYS] motion 任务: 栈 %u 字节，用过后剩余 %u 字节\r\n",
                      static_cast<unsigned>(MOTION_TASK_STACK),
                      static_cast<unsigned>(uxTaskGetStackHighWaterMark(gMotionTask)));
    }
    g_dbg->printf("[SYS] 主循环: 栈剩余 %u 字节 | 任务总数 %u\r\n",
                  static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
                  static_cast<unsigned>(uxTaskGetNumberOfTasks()));
    g_dbg->printf("[SYS] Minima 心跳: %s | SD: %s | WiFi: %s\r\n",
                  statusDisplay.hasRecentHeartbeat(millis()) ? "正常" : "⚠ 超过 3s 无心跳",
                  sdLogger.isReady() ? "已挂载" : "⚠ 不可用",
                  otaManager.isReady() ? "已连接" : (otaManager.isProvisioning() ? "配网热点" : "关闭"));
    // 执行器链路质量：坏帧一直涨说明 2Mbps 那条线在丢字节（帧错位→CRC 失败），
    // ACT_SYNC 只是让它能自愈，真要治得降波特率。前进纠正数应长期为 0。
    g_dbg->printf("[SYS] 执行器链路: 已发 %u 帧 | ESP坏帧 %u | Minima坏帧 %u | Minima最长loop %ums | 前进纠正 %u%s\r\n",
                  static_cast<unsigned>(motionLink.getFramesSent()),
                  static_cast<unsigned>(statusDisplay.getBadFrameCount()),
                  static_cast<unsigned>(statusDisplay.getMinimaBadFrames()),
                  static_cast<unsigned>(statusDisplay.getMinimaLoopMaxMs()),
                  static_cast<unsigned>(gFwdFixCount),
                  statusDisplay.hasMinimaDiag() ? "" : " (执行器固件旧,无回传)");
}

// ── setup ────────────────────────────────────────────────────────────
static void setup() {
    consoleBegin();
    delay(1500);
    printWelcome();

    // ── SPI + CH9434A（必须在超声波之前）──
    g_usb->println("Initializing SPI + CH9434A...");
    {
        constexpr uint8_t  CH_RETRIES     = 5;
        constexpr uint32_t CH_RETRY_DELAY = 500;
        bool ch_ok = false;
        for (uint8_t i = 1; i <= CH_RETRIES; i++) {
            if (ch9434.begin(10000000)) { ch_ok = true; break; }
            g_usb->printf(" [%u/%u failed]", i, CH_RETRIES);
            delay(CH_RETRY_DELAY);
        }
        if (ch_ok) {
            g_usb->println(" OK");
        } else {
            g_usb->println("\r\nCH9434A init failed after 5 attempts — restarting in 2s...");
            delay(2000);
            esp_restart();
        }
    }

    // ── 超声波 UART ──
    g_usb->print("Initializing ultrasonic UARTs... ");
    g_usb->println(ultrasonicMgr.begin() ? "OK" : "FAILED");

    // ── IMU UART（CH9434A UART3）──
    g_usb->print("Initializing IMU (EBIMU-9DOFV6)... ");
    g_usb->println(imuMgr.begin() ? "OK" : "FAILED");

    // ── SD 卡 ──
    g_usb->print("Initializing SD card... ");
    g_usb->println(sdLogger.begin() ? "OK" : "FAILED (logging disabled)");

    // ── 深度传感器 MS5837 ──
    g_usb->print("Initializing MS5837 depth sensor");
    bool depthReady = false;
    for (uint8_t attempt = 1; attempt <= DEPTH_INIT_RETRIES; ++attempt) {
        if (attempt > 1) { g_usb->print(" | retry "); g_usb->print(attempt); }
        if (depthMgr.begin()) { depthReady = true; break; }
        delay(DEPTH_INIT_RETRY_DELAY_MS);
    }
    if (depthReady) {
        g_usb->println("... OK");
    } else {
        g_usb->print("... FAILED (");
        g_usb->print(depthMgr.getStatusText());
        g_usb->println(")");
    }

    // ── WiFi + NTP + OTA ──
    otaManager.begin(false);

    // ── 运动 / 命令模块（HC-12 在 cmdHandler.begin 里初始化）──
    motionLockInit();
    motionLink.begin();
    forwardControl.begin(&motionLink);
    leftTurnControl.begin(&motionLink, /*left=*/true);
    rightTurnControl.begin(&motionLink, /*left=*/false);
    depthController.begin();
    autoNavigator.begin(&forwardControl, &leftTurnControl, &rightTurnControl);
    motionLink.requestParams();   // 读回执行器 EEPROM 里的时序参数

    cmdHandler.begin();
    cmdHandler.setSensorHub(&sensorHub);
    cmdHandler.setStatusDisplay(&statusDisplay);
    cmdHandler.setMotionLink(&motionLink);
    cmdHandler.setForwardControl(&forwardControl);
    cmdHandler.setLeftTurnControl(&leftTurnControl);
    cmdHandler.setRightTurnControl(&rightTurnControl);
    cmdHandler.setDepthController(&depthController);
    cmdHandler.setAutoNavigator(&autoNavigator);
    cmdHandler.setSDLogger(&sdLogger);
    cmdHandler.setEnterTestModeCallback(enterTestMode);
    cmdHandler.setEnterDebugModeCallback(enterDebugMode);
    cmdHandler.setSystemStatusCallback(printSystemStatus);

    // HC-12 已就绪：调试总线切到 USB + HC-12（远程始终能收到反馈）。
    g_dbg = &g_tee;

    // ── 传感器 Hub ──
    sensorHub.setDepthSensorManager(&depthMgr);
    sensorHub.setStatusDisplay(&statusDisplay);
    sensorHub.setUltrasonicManager(&ultrasonicMgr);
    sensorHub.setImuManager(&imuMgr);

    // ── 文件管理网页 + OTA ──
    // 不看 STA 连没连上：机器人自己的热点（192.168.4.1）永远要能烧录。
    // 只有配网门户还开着时才等 —— 那会儿 80 端口在跑配网页面。
    if (!otaManager.isProvisioning()) {
        otaManager.beginWebServer();
        gWebServerStarted = true;
    }

    // ── 启动运动控制任务（此后所有运动时序由它按 5ms 严格推进）──
    xTaskCreatePinnedToCore(motionTask, "motion", MOTION_TASK_STACK, nullptr,
                            MOTION_TASK_PRIO, &gMotionTask, MOTION_TASK_CORE);

    g_dbg->println("系统就绪，输入 h 查看命令列表。\r\n");
}

// ── 循环诊断（2026-09-19 排查前进阀门异常切换时加入）─────────────────────
// 只打到 USB 串口(g_usb)，不走 HC-12，避免诊断本身拖慢循环。
// 1) 单轮 loop 超过 STALL_US：打印各段耗时，定位是哪段阻塞。
// 2) 两轮之间间隔超过 GAP_US：主任务被其它任务抢占/挂起。
// 4) 诊断版 Minima 回传实际执行掩码，与 ESP32 已发掩码核对（查丢帧）。
// 5) motion 任务周期抖动：实际间隔比 5ms 晚 3ms 以上报警。
// 以上只在出问题时打印。SQUID_DIAG_VERBOSE=1 额外打印每 10s 汇总和每帧下发内容。
#define SQUID_LOOP_DIAG 1
#define SQUID_DIAG_VERBOSE 0
#if SQUID_LOOP_DIAG
namespace diag {
enum Sec { S_CMD, S_DEPTH, S_US, S_IMU, S_BATT, S_NAV, S_SD, S_FB, S_DISP, S_OTA, S_N };
const char* const kNames[S_N] = {"cmd", "depth", "us", "imu", "batt", "nav",
                                 "sd", "fb", "disp", "ota"};
// motion 任务：实际周期比设定晚 CTRL_LATE_US 以上就报警。
constexpr uint32_t CTRL_LATE_US = 3000;
int64_t  ctrlLastUs = 0;
uint32_t ctrlMaxLateUs = 0, ctrlLate = 0, ctrlMaxStepUs = 0, ctrlTicks = 0;
constexpr uint32_t STALL_US = 15000;
constexpr uint32_t GAP_US   = 15000;
constexpr int32_t  PHASE_TOL_MS = 20;
constexpr uint32_t SUMMARY_MS = 10000;

int64_t  tLoopStart = 0, tMark = 0, tLastLoopEnd = 0;
uint32_t secUs[S_N] = {};
uint32_t secMaxUs[S_N] = {};
uint32_t maxLoopUs = 0, maxGapUs = 0, stalls = 0, gaps = 0;
uint32_t lastSummaryMs = 0;

inline int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

// motion 任务每拍开头调用：测实际周期。
inline int64_t ctrlBegin() {
    const int64_t t = esp_timer_get_time();
    if (ctrlLastUs != 0) {
        const int64_t late = (t - ctrlLastUs) - static_cast<int64_t>(MOTION_PERIOD_MS) * 1000;
        if (late > 0 && static_cast<uint32_t>(late) > ctrlMaxLateUs) ctrlMaxLateUs = static_cast<uint32_t>(late);
        if (late > CTRL_LATE_US) {
            ++ctrlLate;
            g_usb->printf("[DIAG CTRL] 运动任务周期延迟 %.1f ms (t=%u)\r\n",
                          late / 1000.0, static_cast<unsigned>(millis()));
        }
    }
    ctrlLastUs = t;
    ++ctrlTicks;
    return t;
}
inline void ctrlEnd(int64_t start) {
    const uint32_t step = static_cast<uint32_t>(esp_timer_get_time() - start);
    if (step > ctrlMaxStepUs) ctrlMaxStepUs = step;
}

inline void loopBegin() {
    const int64_t t = esp_timer_get_time();
    if (tLastLoopEnd != 0) {
        const uint32_t gap = static_cast<uint32_t>(t - tLastLoopEnd);
        if (gap > maxGapUs) maxGapUs = gap;
        if (gap > GAP_US) {
            ++gaps;
            g_usb->printf("[DIAG GAP] 主循环被挂起 %u ms (t=%u)\r\n",
                          static_cast<unsigned>(gap / 1000), static_cast<unsigned>(millis()));
        }
    }
    tLoopStart = tMark = t;
}

inline void mark(Sec s) {
    const int64_t t = esp_timer_get_time();
    secUs[s] = static_cast<uint32_t>(t - tMark);
    if (secUs[s] > secMaxUs[s]) secMaxUs[s] = secUs[s];
    tMark = t;
}

// 每次下发给 Minima 的意图变化都打一行（带时间戳），用于核对各动作时序。
uint8_t txLastFwd = 0xFF, txLastDir = 0xFF, txLastPwm = 0xFF;
void traceTx() {
    if (!SQUID_DIAG_VERBOSE) return;
    const uint8_t f = forwardControl.isRunning() ? 1 : 0;
    const uint8_t d = depthController.getBuoyancyDirection();
    const uint8_t p = depthController.getBuoyancyPwm();
    if (f == txLastFwd && d == txLastDir && p == txLastPwm) return;
    txLastFwd = f; txLastDir = d; txLastPwm = p;
    g_usb->printf("[DIAG TX] t=%u 前进=%u 浮沉意图=%u pwm=%u\r\n",
                  static_cast<unsigned>(millis()), static_cast<unsigned>(f),
                  static_cast<unsigned>(d), static_cast<unsigned>(p));
}

inline void loopEnd() {
    const int64_t t = esp_timer_get_time();
    const uint32_t loopUs = static_cast<uint32_t>(t - tLoopStart);
    if (loopUs > maxLoopUs) maxLoopUs = loopUs;
    if (loopUs > STALL_US) {
        ++stalls;
        g_usb->printf("[DIAG STALL] 单轮 %u ms:", static_cast<unsigned>(loopUs / 1000));
        for (int i = 0; i < S_N; ++i) {
            if (secUs[i] >= 1000) {
                g_usb->printf(" %s=%u", kNames[i], static_cast<unsigned>(secUs[i] / 1000));
            }
        }
        g_usb->println(" ms");
    }

    const uint32_t nowMs = millis();
    if (SQUID_DIAG_VERBOSE && nowMs - lastSummaryMs >= SUMMARY_MS) {
        lastSummaryMs = nowMs;
        g_usb->printf("[DIAG] 10s: maxLoop=%.1fms maxGap=%.1fms stall=%u gap=%u\r\n",
                      maxLoopUs / 1000.0, maxGapUs / 1000.0,
                      static_cast<unsigned>(stalls), static_cast<unsigned>(gaps));
        g_usb->printf("[DIAG] 运动任务: 拍数=%u 最长单拍=%.2fms 最大延迟=%.2fms 延迟报警=%u\r\n",
                      static_cast<unsigned>(ctrlTicks), ctrlMaxStepUs / 1000.0,
                      ctrlMaxLateUs / 1000.0, static_cast<unsigned>(ctrlLate));
        ctrlMaxStepUs = ctrlMaxLateUs = 0;
        ctrlTicks = 0;
        g_usb->printf("[DIAG] 链路: 已发帧=%u ESP收坏帧=%u Minima坏帧=%u Minima最长loop=%ums%s\r\n",
                      static_cast<unsigned>(motionLink.getFramesSent()),
                      static_cast<unsigned>(statusDisplay.getBadFrameCount()),
                      static_cast<unsigned>(statusDisplay.getMinimaBadFrames()),
                      static_cast<unsigned>(statusDisplay.getMinimaLoopMaxMs()),
                      statusDisplay.hasMinimaDiag() ? "" : " (旧Minima固件,无回传)");
        g_usb->print("[DIAG] 各段最大耗时(ms):");
        for (int i = 0; i < S_N; ++i) {
            g_usb->printf(" %s=%.1f", kNames[i], secMaxUs[i] / 1000.0);
            secMaxUs[i] = 0;
        }
        g_usb->println();
        maxLoopUs = maxGapUs = 0;
    }
    tLastLoopEnd = esp_timer_get_time();
}
}  // namespace diag
#define DIAG(x) x
#else
#define DIAG(x)
#endif

// ── motion 任务 ──────────────────────────────────────────────────────
// 每 MOTION_PERIOD_MS 严格定时推进全部运动时序并下发 Minima。整拍持 MotionLock，
// 单拍耗时 <1ms（只有状态机运算 + 最多两帧 UART 入缓冲）。
static void motionTask(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastDepthStamp = 0;
    while (true) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTION_PERIOD_MS));
        DIAG(const int64_t t0 = diag::ctrlBegin());
        {
            MotionLockGuard lock;
            const uint32_t nowMs = millis();

            const bool depthFresh = (gDepthSnap.stamp != lastDepthStamp);
            if (depthFresh) lastDepthStamp = gDepthSnap.stamp;
            depthController.update(gDepthSnap.valid, depthFresh, gDepthSnap.depthCm,
                                   gDepthSnap.speedCmS, gDepthSnap.accelCmS2, nowMs);
            // 前进/转向的阀门节拍在执行器本地跑，这里只维护本地意图状态。
            leftTurnControl.update(nowMs);
            rightTurnControl.update(nowMs);

            // 浮沉：方向或 PWM 一变就立刻下发（实时性与原来一致），
            // 另外每 200ms 重发一次当前意图，喂执行器的链路超时保护并自愈丢帧。
            motionLink.applyBuoyancy(depthController.getBuoyancyDirection(),
                                     depthController.getBuoyancyPwm());
            motionLink.refresh(nowMs);
            DIAG(diag::traceTx());
        }
        DIAG(diag::ctrlEnd(t0));
    }
}

// ── 前进意图 ↔ 执行器实际状态：兜底核对 ─────────────────────────────
// MotionLink::refresh() 每 200ms 重申"前进应该在跑"（ACT_SYNC，幂等），已经把
// "起不来"那一侧治好了。这里只兜底反方向：ACT_STOP 丢帧 → 本地已停、执行器还在
// 跑。那一侧不能靠周期重发，因为重发 ACT_STOP 会把平衡窗口反复重置。
//
// 判定用执行器每 500ms 回传的运动状态。要避开两个会合法报 FWD 的窗口：
//   - 普通停止后的平衡（阀门轮流单开，最长约 810ms，掩码非 0 → 算 FWD）
//   - 急停后的 5s 全局平衡（期间 isBalanceLocked() 为真，直接跳过并重新计时）
static void syncForwardIntent(uint32_t nowMs) {
    const bool intent = forwardControl.isRunning();

    // 意图刚变 / 急停平衡锁定中 → 重新计时，等执行器回传跟上再说。
    if (intent != gFwdIntentPrev || cmdHandler.isBalanceLocked()) {
        gFwdIntentPrev     = intent;
        gFwdIntentChangeMs = nowMs;
        return;
    }
    if (intent) return;                       // 该跑那一侧由 refresh() 的 ACT_SYNC 负责
    if (!statusDisplay.hasRecentHeartbeat(nowMs)) return;   // 执行器没心跳就别发了
    if ((statusDisplay.getLastMotionStatus() & 0x01) == 0) return;   // 一致，没事

    if (nowMs - gFwdIntentChangeMs < FWD_CHECK_GRACE_MS) return;
    if (gFwdLastFixMs != 0 && nowMs - gFwdLastFixMs < FWD_CHECK_INTERVAL_MS) return;

    gFwdLastFixMs = nowMs;
    ++gFwdFixCount;
    {
        MotionLockGuard lock;
        motionLink.sendMotion(SUBSYS_FORWARD, ACT_HARD_STOP, 0, 2);
    }
    g_dbg->printf("[SYNC] 本地已停但执行器仍在前进 -> 已补发急停 #%u\r\n",
                  static_cast<unsigned>(gFwdFixCount));
}

// ── loop ─────────────────────────────────────────────────────────────
static void loop() {
    DIAG(diag::loopBegin());
    const uint32_t nowMs = millis();

    cmdHandler.update(nowMs);
    cmdHandler.processSerialInput();
    cmdHandler.processHC12Input();
    DIAG(diag::mark(diag::S_CMD));

    depthMgr.update();
    DIAG(diag::mark(diag::S_DEPTH));
    ultrasonicMgr.update();
    DIAG(diag::mark(diag::S_US));
    imuMgr.update();
    DIAG(diag::mark(diag::S_IMU));
    sensorHub.updateBattery();
    DIAG(diag::mark(diag::S_BATT));

    const bool usFrontValid = ultrasonicMgr.isValid(SENSOR_FRONT);
    const bool usLeftValid  = ultrasonicMgr.isValid(SENSOR_LEFT);
    const bool usRightValid = ultrasonicMgr.isValid(SENSOR_RIGHT);
    const bool depthValid   = depthMgr.isValid();

    // 与 motion 任务交换数据：发布深度快照、自动导航决策、读取日志所需的控制状态。
    // 只做内存读写，持锁时间是微秒级。
    const bool sdDue = sdLogger.hasSession() && nowMs - gLastLogMs >= SD_LOG_INTERVAL_MS;
    bool depthHolding = false, balancing = false, balanceLocked = false;
    float pidRaw = 0.0f, uBase = 0.0f, uResidual = 0.0f, uTotal = 0.0f;
    float target = 0.0f;
    bool outputSaturated = false;
    uint8_t buoyDir = 0, buoyPwm = 0;
    {
        MotionLockGuard lock;
        gDepthSnap.valid     = depthValid;
        gDepthSnap.stamp     = depthMgr.getLastUpdate();
        gDepthSnap.depthCm   = depthMgr.getDepthCm();
        gDepthSnap.speedCmS  = depthMgr.getDepthSpeedCmS();
        gDepthSnap.accelCmS2 = depthMgr.getDepthAccelCmS2();

        handleStartupDepthCalibration(nowMs);
        autoNavigator.update(ultrasonicMgr, nowMs);

        if (sdDue) {
            depthHolding  = depthController.isHoldingTarget();
            pidRaw        = depthController.getPidRawOutput();
            uBase         = depthController.getPidBaseOutput();
            uResidual     = depthController.getResidualOutput();
            uTotal        = depthController.getTotalOutput();
            outputSaturated = depthController.isOutputSaturated();
            target        = depthHolding ? depthController.getTargetDepthCm() : 0.0f;
            buoyDir       = depthController.getBuoyancyDirection();
            buoyPwm       = depthController.getBuoyancyPwm();
            balancing     = depthController.isBalancing();
            balanceLocked = cmdHandler.isBalanceLocked();
        }
    }
    DIAG(diag::mark(diag::S_NAV));

    // SD 传感器日志：20 Hz（仅 TEST session 激活时写入）。
    if (sdDue) {
        const uint32_t dtMs = (gLastLogMs == 0) ? 0 : (nowMs - gLastLogMs);
        gLastLogMs = nowMs;
        const bool imuOk = imuMgr.isValid();
        const float depthErr = (depthHolding && depthValid)
                               ? (target - depthMgr.getDepthCm()) : 0.0f;

        SensorLogRow row;
        row.timestampMs     = nowMs;
        row.dtMs            = dtMs;
        row.robotMode       = (gRobotMode == ROBOT_TEST) ? 1 : 0;
        row.controlMode     = (cmdHandler.getMode() == CommandHandler::MODE_AUTO) ? 1 : 0;
        row.depthValid      = depthValid;
        row.imuValid        = imuOk;
        row.batteryV        = sensorHub.getBatteryVoltage();
        row.targetDepthCm   = target;
        row.filteredDepthCm = depthValid ? depthMgr.getDepthCm()        : 0.0f;
        row.depthSpeedCmS   = depthValid ? depthMgr.getDepthSpeedCmS()  : 0.0f;
        row.depthAccelCmS2  = depthValid ? depthMgr.getDepthAccelCmS2() : 0.0f;
        row.roll            = imuOk ? imuMgr.getRoll()  : 0.0f;
        row.pitch           = imuOk ? imuMgr.getPitch() : 0.0f;
        row.yaw             = imuOk ? imuMgr.getYaw()   : 0.0f;
        row.gyroX           = imuOk ? imuMgr.getGyroX() : 0.0f;
        row.gyroY           = imuOk ? imuMgr.getGyroY() : 0.0f;
        row.gyroZ           = imuOk ? imuMgr.getGyroZ() : 0.0f;
        row.frontCm         = usFrontValid ? ultrasonicMgr.getDistance(SENSOR_FRONT) / 10.0f : -1.0f;
        row.leftCm          = usLeftValid  ? ultrasonicMgr.getDistance(SENSOR_LEFT)  / 10.0f : -1.0f;
        row.rightCm         = usRightValid ? ultrasonicMgr.getDistance(SENSOR_RIGHT) / 10.0f : -1.0f;
        row.depthErrCm      = depthErr;
        row.pidRaw          = pidRaw;
        row.uBase           = uBase;
        row.uResidual       = uResidual;
        row.uTotal          = uTotal;
        row.outputSaturated = outputSaturated;
        row.buoyancyDir     = buoyDir;
        row.buoyancyPwm     = buoyPwm;
        row.balancing       = balancing;
        row.emergencyStop   = balanceLocked;
        sdLogger.logSensor(row);
    }
    DIAG(diag::mark(diag::S_SD));

    statusDisplay.processMinimaFeedback();

    // PC 콘솔의 별도 control-*.csv용 진단 프레임. 기존 센서/이벤트 로그는
    // 건드리지 않고, 0.5초 주기로 제어에 필요한 값만 별도 전송한다.
    if (nowMs - gLastControlTelemetryMs >= CONTROL_TELEMETRY_INTERVAL_MS) {
        gLastControlTelemetryMs = nowMs;

        DepthSnapshot controlDepth;
        bool targetValid = false;
        float targetDepth = 0.0f;
        float pidOutput = 0.0f;
        float pidBase = 0.0f;
        bool forwardActive = false;
        uint8_t buoyancyDirection = BUOYANCY_STOP;
        uint8_t buoyancyPwm = 0;
        {
            MotionLockGuard lock;
            controlDepth = gDepthSnap;
            targetValid = depthController.isHoldingTarget();
            targetDepth = targetValid ? depthController.getTargetDepthCm() : 0.0f;
            pidOutput = depthController.getControlOutput();
            pidBase = depthController.getPidBaseOutput();
            forwardActive = forwardControl.isRunning();
            buoyancyDirection = depthController.getBuoyancyDirection();
            buoyancyPwm = depthController.getBuoyancyPwm();
        }

        // Minima 원본 펌웨어는 상태 마스크에 A/B만 회신하고 E/F는 회신하지
        // 않는다. A/B는 회신값을 우선 사용하고, E/F는 ESP32가 보낸 부력
        // 명령으로 추정해 기록한다. 따라서 E/F는 실제 핀 피드백이 아니다.
        uint16_t estimatedValveMask = 0;
        if (forwardControl.isRunning()) {
            // Minima가 로컬 시퀀스로 A/B를 교대하므로, ESP32에서는 전진 의도
            // 기준으로 두 밸브가 활성인 것으로 표시한다.
            estimatedValveMask |= ACT_FORWARD_VALVE_A | ACT_FORWARD_VALVE_B;
        }

        if (buoyancyDirection == cal::BUOY_RISE) {
            // 원본 Minima 프로토콜에서 BUOY_RISE는 E/F 모두 여는 명령이다.
            estimatedValveMask |= ACT_BUOYANCY_VALVE_E | ACT_BUOYANCY_VALVE_F;
        } else if (buoyancyDirection == BUOYANCY_BALANCE) {
            // Minima 기본 평형 주기(500ms)를 기준으로 한 추정값이다.
            const bool phaseF = ((millis() / 500U) & 1U) != 0;
            estimatedValveMask |= phaseF ? ACT_BUOYANCY_VALVE_F : ACT_BUOYANCY_VALVE_E;
        }

        const bool minimaMaskValid = statusDisplay.hasMinimaDiag();
        const uint16_t reportedMask = minimaMaskValid
            ? static_cast<uint16_t>(
                  (statusDisplay.getMinimaMask() &
                   (ACT_FORWARD_VALVE_A | ACT_FORWARD_VALVE_B)) |
                  (estimatedValveMask &
                   (ACT_BUOYANCY_VALVE_E | ACT_BUOYANCY_VALVE_F)))
            : estimatedValveMask;
        // 기존 PC 콘솔 수집 경로와 원래 제어 데이터 항목을 유지한다.
        g_dbg->printf("[CTRL] depth_valid=%u depth=%.3f vz=%.3f az=%.3f "
                      "target_valid=%u target=%.3f pid=%.3f "
                      "pid_base=%.3f buoyancy_pwm=%u "
                      "mask_valid=%u mask=%u forward_active=%u\r\n",
                      controlDepth.valid ? 1U : 0U,
                      controlDepth.depthCm,
                      controlDepth.speedCmS,
                      controlDepth.accelCmS2,
                      targetValid ? 1U : 0U,
                      targetDepth,
                      pidOutput,
                      pidBase,
                      static_cast<unsigned>(buoyancyPwm),
                      1U,
                      static_cast<unsigned>(reportedMask),
                      forwardActive ? 1U : 0U);
    }

    syncForwardIntent(nowMs);
    DIAG(diag::mark(diag::S_FB));
    sensorHub.displayAll();
    DIAG(diag::mark(diag::S_DISP));
    otaManager.handle();
    DIAG(diag::mark(diag::S_OTA));

    // 配网门户退出后把文件管理网页/OTA 补起来。
    if (!gWebServerStarted && !otaManager.isProvisioning()) {
        otaManager.beginWebServer();
        gWebServerStarted = true;
    }
    DIAG(diag::loopEnd());
}

// ── ESP-IDF 入口 ─────────────────────────────────────────────────────
extern "C" void app_main(void) {
    setup();
    while (true) {
        loop();
        vTaskDelay(1);   // 让出 CPU 给其它任务/看门狗（约 1ms 一轮）
    }
}
