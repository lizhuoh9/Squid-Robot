/**********************************************************************
 * DepthController.h
 *
 * 定深控制器：用滤波后的深度状态和连续浮力命令做预测式定深控制。
 *********************************************************************/

#ifndef SQUID_DEPTH_CONTROLLER_H
#define SQUID_DEPTH_CONTROLLER_H

#include <cstdint>

#include "Board.h"

class DepthController {
public:
    DepthController();

    void begin();
    // depthFresh：本轮深度是否是"新采样"。非阻塞采样后主循环 ~1kHz，但深度仍
    // ~12Hz 才更新一次；定深 PID 只在 depthFresh 时推进，保持与原来一致的控制
    // 频率、避免微分项在采样边界尖峰。手动/平衡/深度失效逻辑不受此限，每轮都跑。
    void update(bool depthValid,
                bool depthFresh,
                float filteredDepthCm,
                float depthSpeedCmS,
                float depthAccelCmS2,
                uint32_t nowMs);

    void setTargetDepth(float targetDepthCm);
    void holdCurrentDepth();
    void clearTarget();
    bool isHoldingTarget() const;
    bool isBalancing() const { return _balancing; }

    void manualAscend();
    void manualDescend();
    void manualStop();
    void forceBalance();

    void resetAfterCalibration();

    float getFilteredDepthCm() const;
    float getSpeedCmS() const;
    float getAccelerationCmS2() const;
    float getTargetDepthCm() const;
    float getControlOutput() const;

    uint8_t getBuoyancyDirection() const;   // 意图：停/上浮/下沉/平衡
    uint8_t getBuoyancyPwm() const;         // 泵力度（执行器按自己的参数决定何时给力）
    uint8_t getManualDirection() const;

private:
    void adaptivePID(float err, float derr, float spd, float dt);
    void speedLimiter(float& u, float err);
    void applyBuoyancyOutput(float u);
    void updateIntent(bool depthValid, bool depthFresh, float filteredDepthCm,
                      float depthSpeedCmS, float depthAccelCmS2, uint32_t nowMs);
    void stopBuoyancyOutput();

    bool _holdingTarget;

    float _targetDepthCm;
    float _depthFilt;
    float _speedCmS;
    float _accelCmS2;
    float _integ;
    float _errPrev;
    float _derivPrev;
    float _controlOutput;

    float _kpBase;
    float _kiBase;
    float _kdBase;
    float _kp;
    float _ki;
    float _kd;

    uint8_t  _manualDirection;
    uint8_t  _buoyancyDirection;
    uint8_t  _buoyancyPwm;
    uint32_t _lastControlUpdateMs;
    bool     _balancing;
    uint32_t _balanceStartMs;
    uint32_t _balanceEndMs;
};

#endif  // SQUID_DEPTH_CONTROLLER_H
