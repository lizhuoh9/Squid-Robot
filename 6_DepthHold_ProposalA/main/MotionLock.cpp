/**********************************************************************
 * MotionLock.cpp
 *********************************************************************/

#include "MotionLock.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {
SemaphoreHandle_t s_mtx = nullptr;
}  // namespace

void motionLockInit() {
    if (!s_mtx) s_mtx = xSemaphoreCreateMutex();   // 普通互斥锁，带优先级继承
}

void motionLock() {
    if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY);
}

void motionUnlock() {
    if (s_mtx) xSemaphoreGive(s_mtx);
}
