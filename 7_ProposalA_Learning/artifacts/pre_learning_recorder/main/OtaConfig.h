/**********************************************************************
 * OtaConfig.h
 *
 * OTA / WiFi / 热点配置。值全部在 OtaConfig.local.h 里，那个文件不进 git
 * （见根目录 .gitignore）；第一次编译前把同目录的 OtaConfig.local.h.example
 * 复制成 OtaConfig.local.h 再填。
 *********************************************************************/

#ifndef SQUID_OTA_CONFIG_H
#define SQUID_OTA_CONFIG_H

#if __has_include("OtaConfig.local.h")
#include "OtaConfig.local.h"
#else
// 不放默认值：这个文件进 git，写在这里的密码等于公开。
#error "OtaConfig.local.h is missing: copy OtaConfig.local.h.example to OtaConfig.local.h and fill in the values"
#endif

#endif  // SQUID_OTA_CONFIG_H
