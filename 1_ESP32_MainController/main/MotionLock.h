/**********************************************************************
 * MotionLock.h
 *
 * 运动控制对象（ForwardControl / LeftTurnControl / RightTurnControl /
 * DepthController / AutoNavigator / MotionLink）的共享互斥锁。
 *
 * 运动时序由高优先级 motion 任务每 5ms 严格推进；主循环（命令、自动导航、
 * 深度快照、SD 记录）改这些对象时必须持锁。持锁区只做很短的操作——
 * 不要在锁内做 WiFi 连接、HC-12 改信道等慢操作，否则会卡住阀门时序。
 *********************************************************************/

#ifndef SQUID_MOTION_LOCK_H
#define SQUID_MOTION_LOCK_H

void motionLockInit();
void motionLock();
void motionUnlock();

class MotionLockGuard {
public:
    MotionLockGuard() { motionLock(); }
    ~MotionLockGuard() { motionUnlock(); }
    MotionLockGuard(const MotionLockGuard&) = delete;
    MotionLockGuard& operator=(const MotionLockGuard&) = delete;
};

#endif  // SQUID_MOTION_LOCK_H
