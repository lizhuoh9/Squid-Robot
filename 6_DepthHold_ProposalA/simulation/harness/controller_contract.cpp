#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <DepthController.h>
#include <Calibration.h>

uint32_t g_simMillis = 0;

static void sample(DepthController& c, uint32_t ms, float d, float v = 0.0f,
                   float a = 0.0f, bool valid = true, bool fresh = true) {
    g_simMillis = ms;
    c.update(valid, fresh, d, v, a, ms);
}

int main() {
    DepthController c;
    c.begin();
    c.setTargetDepth(40.0f);
    sample(c, 85, 40.0f);
    sample(c, 170, 40.0f);
    assert(std::fabs(c.getPidRawOutput()) < 1e-6f);
    assert(std::fabs(c.getPidBaseOutput() - 1.1f) < 1e-6f);
    assert(std::fabs(c.getControlOutput() + 100.0f * 1.1f / 4.5f) < 1e-4f);
    assert(c.getResidualOutput() == 0.0f);

    // Pulse requests must remain demanded during their off phase. Low duty is
    // emitted as PWM 80, never as a physically unsupported low PWM value.
    int onTicks = 0, offTicks = 0;
    for (uint32_t ms = 1000; ms < 2000; ms += 5) {
        sample(c, ms, 40.0f, 0.0f, 0.0f, true, false);
        assert(c.getBuoyancyPwm() == 0 || c.getBuoyancyPwm() == 80);
        if (c.getBuoyancyPwm() == 80) {
            ++onTicks;
            assert(c.getBuoyancyDirection() == cal::BUOY_RISE);
        } else {
            ++offTicks;
        }
        assert(c.getControlOutput() < 0.0f);
    }
    assert(onTicks >= 154 && onTicks <= 158 && offTicks > 0);

    // The integral represents trim and survives a target change.
    c.resetAfterCalibration();
    c.setTargetDepth(40.0f);
    for (uint32_t ms = 85; ms <= 1020; ms += 85) sample(c, ms, 41.0f);
    sample(c, 1105, 41.0f);
    const float oldBias = c.getPidBaseOutput();
    c.setTargetDepth(41.0f);
    sample(c, 1190, 41.0f);
    assert(c.getPidBaseOutput() >= oldBias && c.getPidBaseOutput() > 1.135f);

    c.manualAscend();
    sample(c, 1300, 0.0f, 0.0f, 0.0f, false);
    assert(c.getBuoyancyDirection() == cal::BUOY_RISE && c.getBuoyancyPwm() == 255);
    c.manualStop();
    c.manualDescend();
    assert(c.getBuoyancyDirection() == cal::BUOY_SINK && c.getBuoyancyPwm() == 255);
    c.manualStop();
    assert(c.getBuoyancyPwm() == 0);

    c.setTargetDepth(40.0f);
    sample(c, 1400, 40.0f);
    sample(c, 1485, 40.0f);
    sample(c, 1500, 40.0f, 0.0f, 0.0f, false);
    assert(c.getBuoyancyPwm() == 0 && c.getControlOutput() == 0.0f);
    sample(c, 1585, std::numeric_limits<float>::quiet_NaN());
    assert(c.getBuoyancyPwm() == 0);
    c.setTargetDepth(40.0f);
    sample(c, 1670, 100.0f);
    assert(!c.isHoldingTarget() && c.getBuoyancyPwm() == 0);

    c.forceBalance();
    sample(c, 2000, 40.0f);
    assert(c.isBalancing() && c.getBuoyancyDirection() == BUOYANCY_BALANCE);
    sample(c, 7000, 40.0f);
    assert(!c.isBalancing() && c.getBuoyancyPwm() == 0);

    DepthController a, b;
    a.begin(); b.begin(); a.setTargetDepth(40.0f); b.setTargetDepth(40.0f);
    sample(a, 85, 41.0f, 0.5f, 10.0f); sample(b, 85, 41.0f, 0.5f, -10.0f);
    sample(a, 170, 41.0f, 0.5f, 10.0f); sample(b, 170, 41.0f, 0.5f, -10.0f);
    assert(a.getControlOutput() == b.getControlOutput());
    std::puts("PASS: acceleration control, zero residual, pulse duty, integral retention, manual mapping, invalid sensor, max depth, balance, acceleration ignored");
}
