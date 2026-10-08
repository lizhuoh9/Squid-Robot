// Host-build stub: simulated millisecond clock instead of esp_timer.
#ifndef SQUID_SYS_STUB_H
#define SQUID_SYS_STUB_H
#include <cstdint>
extern uint32_t g_simMillis;
inline uint32_t millis() { return g_simMillis; }
#endif
