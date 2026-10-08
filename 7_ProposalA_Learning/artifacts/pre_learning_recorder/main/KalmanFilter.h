/**********************************************************************
 * KalmanFilter.h
 *
 * 三状态深度滤波器（位置 / 速度 / 加速度）。纯数学，无平台依赖。
 *********************************************************************/

#ifndef SQUID_KALMAN_FILTER_H
#define SQUID_KALMAN_FILTER_H

#include <cstdint>

class KalmanFilter {
public:
    KalmanFilter();

    void reset(float position = 0.0f, float velocity = 0.0f, float acceleration = 0.0f);
    void update(float measurement, float dt);

    float getPosition() const;
    float getVelocity() const;
    float getAcceleration() const;

private:
    float _x[3];
    float _p[3][3];
    float _q[3];
    float _r;
};

#endif  // SQUID_KALMAN_FILTER_H
