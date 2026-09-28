/**********************************************************************
 * SdCard.h
 *
 * SD 卡（SPI3/HSPI）挂载与路径工具。原固件用 Arduino SD 库，这里用
 * esp_vfs_fat_sdspi_mount 挂到 /sdcard，之后各处用标准 POSIX 文件 API。
 * SDLogger 和 OtaManager 的网页文件管理共用本层。
 *********************************************************************/

#ifndef SQUID_SD_CARD_H
#define SQUID_SD_CARD_H

#include <cstddef>
#include <cstdint>

// FATFS 挂载点。逻辑路径（如网页里以 "/" 为根、SDLogger 的 "/<session>"）
// 都会在这里加上该前缀映射到真实文件系统路径。
extern const char* const SD_MOUNT_POINT;

// 挂载 SD 卡（幂等）。成功返回 true。
bool sdCardMount();
bool sdCardMounted();

// 诊断：卸载后按多组参数（内部上拉 / 上电等待 / 时钟）反复尝试挂载，逐次打印
// 结果与错误码；最后保留第一个成功的组合处于挂载状态。返回是否有组合成功。
bool sdCardDiagnose();

// SD 卡总容量（字节）。
uint64_t sdCardSizeBytes();

// 把逻辑路径（以 "/" 为根）拼成真实 fs 路径写入 dst。
//   "/"            -> "/sdcard"
//   "/a/b.csv"     -> "/sdcard/a/b.csv"
// 返回 dst。
char* sdFsPath(char* dst, size_t dstLen, const char* logicalPath);

#endif  // SQUID_SD_CARD_H
