/**********************************************************************
 * KalmanFilter.cpp
 *
 * 恒加速度卡尔曼滤波：深度、垂直速度、垂直加速度。
 *********************************************************************/

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    _q[0] = 0.02f;
    _q[1] = 0.15f;
    _q[2] = 0.80f;
    _r = 0.35f;
    reset();
}

void KalmanFilter::reset(float position, float velocity, float acceleration) {
    _x[0] = position;
    _x[1] = velocity;
    _x[2] = acceleration;

    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            _p[row][col] = row == col ? 1.0f : 0.0f;
        }
    }
}

void KalmanFilter::update(float measurement, float dt) {
    const float dt2 = dt * dt;
    const float halfDt2 = 0.5f * dt2;
    const float f[3][3] = {
        {1.0f, dt, halfDt2},
        {0.0f, 1.0f, dt},
        {0.0f, 0.0f, 1.0f}
    };

    float xPred[3];
    for (uint8_t row = 0; row < 3; ++row) {
        xPred[row] = 0.0f;
        for (uint8_t col = 0; col < 3; ++col) {
            xPred[row] += f[row][col] * _x[col];
        }
    }

    float fp[3][3];
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            fp[row][col] = 0.0f;
            for (uint8_t k = 0; k < 3; ++k) {
                fp[row][col] += f[row][k] * _p[k][col];
            }
        }
    }

    float pPred[3][3];
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            pPred[row][col] = 0.0f;
            for (uint8_t k = 0; k < 3; ++k) {
                pPred[row][col] += fp[row][k] * f[col][k];
            }
            if (row == col) {
                pPred[row][col] += _q[row];
            }
        }
    }

    const float innovation = measurement - xPred[0];
    const float s = pPred[0][0] + _r;
    float k[3];
    for (uint8_t row = 0; row < 3; ++row) {
        k[row] = pPred[row][0] / s;
    }

    for (uint8_t row = 0; row < 3; ++row) {
        _x[row] = xPred[row] + k[row] * innovation;
    }

    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            _p[row][col] = pPred[row][col] - k[row] * pPred[0][col];
        }
    }
}

float KalmanFilter::getPosition() const { return _x[0]; }
float KalmanFilter::getVelocity() const { return _x[1]; }
float KalmanFilter::getAcceleration() const { return _x[2]; }
