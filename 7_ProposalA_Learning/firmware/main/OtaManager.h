/**********************************************************************
 * OtaManager.h
 *
 * 网络管理器（原生 ESP-IDF 版）：
 *   - WiFi STA：用 NVS 保存的凭据连接（失败则关 WiFi 继续启动）
 *   - 配网热点 + Captive Portal（可选，beginProvisioningPortal）
 *   - NTP 时间同步（esp_sntp）
 *   - HTTP 服务（esp_http_server）：SD 文件管理 + OTA 上传
 *     （网页控制台已删除，控制只走 USB 串口和 HC-12）
 *
 * 原固件用 ArduinoOTA（espota 协议）。原生版改为 HTTP 固件上传
 * （POST /ota，走 esp_ota），是等价的“无线烧录”能力，见 README。
 *
 * HTTP 接口：
 *   GET    / 或 /console      文件管理页面
 *   GET    /files?path=/      列目录 JSON
 *   GET    /file?path=/x/y    下载文件
 *   DELETE /file?path=/x/y    递归删除文件/目录
 *   POST   /upload?path=/dir  multipart 上传
 *   POST   /ota               固件上传（application/octet-stream）
 *********************************************************************/

#ifndef SQUID_OTA_MANAGER_H
#define SQUID_OTA_MANAGER_H

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <functional>
#include <string>

#include "Output.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class OtaManager {
public:
    OtaManager();

    void begin(bool underwaterMode = false);
    void handle();

    bool isReady() const;
    void getLocalIpStr(char* buf, size_t len) const;

    bool syncNtp();
    bool hasTime() const;
    void getSessionFolderName(char* buf, size_t len) const;

    // 启动 HTTP 服务：SD 文件管理页（/ 或 /console）+ OTA 上传（/ota）。
    void beginWebServer();
    // 配网门户是否还开着（它占着 80 端口，此时别起文件管理/OTA 的路由）。
    bool isProvisioning() const { return _apActive; }

private:
    bool     _ready;
    bool     _consoleActive;
    bool     _ntpSynced;
    bool     _reconnectEnabled;
    uint32_t _nextReconnectMs;
    time_t   _ntpEpoch;
    uint32_t _ntpSyncMs;

    // 配网热点状态。
    bool          _apActive;
    volatile bool _provisionPending;
    std::string   _pendingSsid;
    std::string   _pendingPass;

    httpd_handle_t _httpd;

    bool beginNvsMode();
    bool connectSta(const char* ssid, const char* pass, uint32_t timeoutMs);
    void finishStaSetup();
    void registerRoutes();

public:
    // 配网热点：连不上 WiFi 时开 SquidRobot-Setup + 强制门户（不阻塞主循环）。
    // 把机器人自己的热点拉起来（APSTA），无论外部 WiFi 连没连上。
    void beginAlwaysOnAp();
    void beginProvisioningPortal();
    void stopProvisioningPortal();
    // 供 HTTP handler 调用：记下待连接的凭据，主循环里再连（避免在 httpd 任务里拆自己）。
    void queueProvisionCreds(const char* ssid, const char* pass);

    // HTTP 处理（静态 handler 转发到实例方法）。
    friend class OtaHttp;
};

#endif  // SQUID_OTA_MANAGER_H
