/**********************************************************************
 * OtaConfig.h
 *
 * OTA / 主机名默认配置。需要私密配置可新建 OtaConfig.local.h 覆盖。
 *********************************************************************/

#ifndef SQUID_OTA_CONFIG_H
#define SQUID_OTA_CONFIG_H

#if __has_include("OtaConfig.local.h")
#include "OtaConfig.local.h"
#else

constexpr char OTA_HOSTNAME[] = "squid-robot";
constexpr char OTA_PASSWORD[] = "12345678";

// ── 机器人自带的常开热点 ──────────────────────────────────────────
// 主控焊死在壳体里，USB 够不着，OTA 是唯一入口。所以不能把这个入口寄托在
// "某台电脑的热点开着"或"实验室路由器可达"上 —— 机器人自己放一个热点，
// 谁要烧录就连它，地址永远是 http://192.168.4.1/，跟外部网络无关。
// 外部 WiFi（STA）照常连，连上了就多一条路，连不上也不影响烧录。
constexpr char AP_SSID[]     = "SquidRobot";
constexpr char AP_PASSWORD[] = "squid12345";   // WPA2 至少 8 位；实验室公用，别留空


constexpr uint32_t OTA_CONNECT_TIMEOUT_MS = 15000;

// 指定 WiFi：填了就【优先】连它，连不上再退回 NVS 里存的上一个网络。
// 之所以是"优先"而不是"唯一"：主控焊死在壳体里、USB 够不着，密码打错一个字
// 就再也连不上了。优先+回退的组合让改网络这件事没有代价 —— 新的连不上，
// 自动回到旧的，OTA 入口始终在。
// 留空则完全走 NVS（配网门户配过一次就存在那儿了）。
constexpr char WIFI_FALLBACK_SSID[] = "";
constexpr char WIFI_FALLBACK_PASS[] = "";

#endif

#endif  // SQUID_OTA_CONFIG_H
