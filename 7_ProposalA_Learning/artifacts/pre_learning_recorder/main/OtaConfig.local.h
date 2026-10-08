/**********************************************************************
 * OtaConfig.local.h  —— 本地私密配置（覆盖 OtaConfig.h 的默认值）
 *
 * 存在这个文件时，OtaConfig.h「只用这里」，所以用到的常量都要定义齐。
 *
 * 现在配网有两条路：
 *   1)【推荐】热点配网：机器人连不上 WiFi 会自动开热点 "SquidRobot-Setup"，
 *      手机连上后浏览器打开任意网页会弹配网页（或直接开 http://192.168.4.1/）。
 *      配一次存进 NVS，之后自动连。—— 不用改这个文件、不用重烧。
 *   2)【可选】下面填死一个 WiFi，开机自动连（适合固定网络、免手动配网）。
 *      不用就保持空字符串，会直接走热点配网。
 *********************************************************************/

#ifndef SQUID_OTA_CONFIG_LOCAL_H
#define SQUID_OTA_CONFIG_LOCAL_H

// 指定 WiFi：填了就【优先】连它，连不上再退回 NVS 里存的上一个网络。
// 之所以是"优先"而不是"唯一"：主控焊死在壳体里、USB 够不着，密码打错一个字
// 就再也连不上了。优先+回退的组合让改网络这件事没有代价 —— 新的连不上，
// 自动回到旧的，OTA 入口始终在。
// 留空则完全走 NVS（配网门户配过一次就存在那儿了）。
// 实验室 2.4G（ESP32-S3 连不了 5GHz，所以不能用 ssbrl5G）。
constexpr char WIFI_FALLBACK_SSID[] = "ssbrl";
constexpr char WIFI_FALLBACK_PASS[] = "brl12345";

// ── 一般不用动 ────────────────────────────────────────────────────
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

#endif  // SQUID_OTA_CONFIG_LOCAL_H
