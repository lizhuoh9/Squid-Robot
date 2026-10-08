// Closed-loop host simulation of the squid robot depth loop.
// Links the real firmware DepthController.cpp + KalmanFilter.cpp (host stubs for Board.h/Sys.h).
//
// Plant (depth d, velocity v, net buoyancy W; all positive downward, cm / s):
//   W' = Ks*sink(t-L) - Kr*rise(t-L) - (W - W_rest(d))/tauW,  W_rest(d) = aRest + alpha*(d - 40)
//   v' = W - cq*v|v|,   d' = v
// Sensor: MS5837 sample every 85 ms + N(0, 0.05 cm) -> firmware KalmanFilter (same Q, R, dt clamp).
// Actuator: Minima BuoySeq -- STOP/SINK keep E/F closed, RISE opens them; any valve change locks
//           the pump for 200 ms; duty = PWM/255; no flow below PWM 80.
//
// usage: sim calib
//        sim eval  Ks Kr tauW L aRest alpha seed
//        sim hold  Ks Kr tauW L aRest alpha seed [trace.csv]
//        sim trace Ks Kr tauW L aRest alpha seed out.csv
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "Calibration.h"
#include "DepthController.h"
#include "KalmanFilter.h"

uint32_t g_simMillis = 0;

namespace {
constexpr double DT = 0.005;
constexpr uint32_t TICK_MS = 5;
constexpr uint32_t SAMPLE_MS = 85;
constexpr uint32_t VALVE_GUARD_MS = 200;
#ifndef SIM_PUMP_FLOOR_PWM
#define SIM_PUMP_FLOOR_PWM 80
#endif
constexpr double PUMP_FLOOR = SIM_PUMP_FLOOR_PWM / 255.0;

struct PlantParams {
    double Ks, Kr, tauW, L, aRest, alpha;
    double dRef = 40.0, cq = 0.02, noise = 0.05;
};

class Plant {
public:
    Plant(const PlantParams& p, double d0, unsigned seed)
        : p_(p), d_(d0), rng_(seed), noise_(0.0, p.noise) {
        n_ = std::max<size_t>(1, static_cast<size_t>(std::lround(p.L / DT)));
        buf_.assign(n_, 0.0);
    }
    void step(double signedDuty) {
        if (signedDuty != 0.0 && std::fabs(signedDuty) < PUMP_FLOOR - 1e-6) signedDuty = 0.0;
        const double u = buf_[k_ % n_];
        buf_[k_ % n_] = signedDuty;
        ++k_;
        const double pump = u > 0.0 ? p_.Ks * u : p_.Kr * u;
        const double wRest = p_.aRest + p_.alpha * (d_ - p_.dRef);
        W_ += (pump - (W_ - wRest) / p_.tauW) * DT;
        const double a = W_ - p_.cq * v_ * std::fabs(v_);
        v_ += a * DT;
        d_ += v_ * DT;
        if (d_ < 10.5) { d_ = 10.5; if (v_ < 0.0) v_ = 0.0; }
        if (d_ > 77.0) { d_ = 77.0; if (v_ > 0.0) v_ = 0.0; }
    }
    double measure() { return d_ + noise_(rng_); }
    double depth() const { return d_; }

private:
    PlantParams p_;
    double d_, v_ = 0.0, W_ = 0.0;
    std::vector<double> buf_;
    size_t n_ = 1, k_ = 0;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_;
};

struct Sample { double t, d, target, vel; int intent; double applied; bool sat; };
struct RunResult { std::vector<Sample> log; long valveChanges = 0; double dutySum = 0.0; long ticks = 0; double endS = 0; };
using Schedule = std::vector<std::pair<double, double>>;   // (time s, target cm); last entry = end time

RunResult run(const PlantParams& pp, const Schedule& sched, double d0, unsigned seed) {
    Plant plant(pp, d0, seed);
    KalmanFilter kf;
    DepthController ctrl;
    g_simMillis = 0;
    ctrl.begin();
    RunResult rr;
    bool kfInit = false;
    uint32_t nextSampleMs = 0, lastSampleMs = 0, stamp = 0, lastStamp = 0;
    uint8_t valves = 0;
    uint32_t valveChangeMs = 0;
    size_t si = 0;
    double target = sched.front().second;
    const uint32_t endMs = static_cast<uint32_t>(sched.back().first * 1000.0);
    uint32_t nextLogMs = 0;
    for (uint32_t now = 0; now < endMs; now += TICK_MS) {
        g_simMillis = now;
        while (si + 1 < sched.size() && now >= static_cast<uint32_t>(sched[si].first * 1000.0)) {
            target = sched[si].second;
            ctrl.setTargetDepth(static_cast<float>(target));
            ++si;
        }
        if (now >= nextSampleMs) {
            const double z = plant.measure();
            if (!kfInit) {
                kf.reset(static_cast<float>(z), 0.0f, 0.0f);
                kfInit = true;
            } else {
                double dt = (now - lastSampleMs) * 0.001;
                dt = std::min(std::max(dt, 0.01), 0.20);
                kf.update(static_cast<float>(z), static_cast<float>(dt));
            }
            lastSampleMs = now;
            nextSampleMs = now + SAMPLE_MS;
            stamp = now + 1;
        }
        const bool fresh = (stamp != lastStamp);
        lastStamp = stamp;
        ctrl.update(true, fresh, kf.getPosition(), kf.getVelocity(), kf.getAcceleration(), now);

        // Minima BuoySeq
        const uint8_t dir = ctrl.getBuoyancyDirection();
        const uint8_t pwm = ctrl.getBuoyancyPwm();
        uint8_t want = 0;
        if (dir == cal::BUOY_RISE) want = 3;
        else if (dir == BUOYANCY_BALANCE) want = ((now / 500U) & 1U) ? 2 : 1;
        if (want != valves) { valves = want; valveChangeMs = now; ++rr.valveChanges; }
        double duty = 0.0;
        if ((dir == cal::BUOY_SINK || dir == cal::BUOY_RISE) && pwm > 0 && now - valveChangeMs >= VALVE_GUARD_MS)
            duty = pwm / 255.0;
        const double signedDuty = (dir == cal::BUOY_SINK) ? duty : -duty;
        plant.step(signedDuty);
        rr.dutySum += duty;
        ++rr.ticks;

        if (now >= nextLogMs) {
            int intent = 0;
            if (dir == cal::BUOY_SINK) intent = pwm;
            else if (dir == cal::BUOY_RISE) intent = -static_cast<int>(pwm);
            rr.log.push_back({now * 0.001, plant.depth(), target, kf.getVelocity(), intent, signedDuty, ctrl.isOutputSaturated()});
            nextLogMs = now + 500;
        }
    }
    rr.endS = endMs * 0.001;
    return rr;
}

struct StepMetrics { double rms, w1, w05, over, valvePerMin, dutyPct; };

StepMetrics stepMetrics(const RunResult& rr, const Schedule& sched, double settle) {
    std::vector<double> e;
    double over = -1e9;
    for (size_t i = 0; i + 1 < sched.size(); ++i) {
        const double t0 = sched[i].first, t1 = sched[i + 1].first, tgt = sched[i].second;
        double dStart = NAN;
        double segOver = -1e9;
        for (const auto& s : rr.log) {
            if (s.t < t0 || s.t >= t1) continue;
            if (std::isnan(dStart)) dStart = s.d;
            if (s.t >= t0 + settle) e.push_back(tgt - s.d);
        }
        const double sgn = (tgt > dStart) ? 1.0 : -1.0;
        for (const auto& s : rr.log)
            if (s.t >= t0 && s.t < t1) segOver = std::max(segOver, sgn * (s.d - tgt));
        over = std::max(over, segOver);
    }
    double ss = 0; int n1 = 0, n05 = 0;
    for (double x : e) { ss += x * x; if (std::fabs(x) < 1.0) ++n1; if (std::fabs(x) < 0.5) ++n05; }
    StepMetrics m;
    m.rms = std::sqrt(ss / e.size());
    m.w1 = 100.0 * n1 / e.size();
    m.w05 = 100.0 * n05 / e.size();
    m.over = over;
    m.valvePerMin = rr.valveChanges / (rr.endS / 60.0);
    m.dutyPct = 100.0 * rr.dutySum / rr.ticks;
    return m;
}

struct HoldMetrics { double period, p2p, vamp, risePct, sinkPct, idlePct, rms, w1; };

HoldMetrics holdMetrics(const RunResult& rr, double tFrom, double target) {
    std::vector<const Sample*> seg;
    for (const auto& s : rr.log) if (s.t > tFrom) seg.push_back(&s);
    std::vector<double> zc;
    double emin = 1e9, emax = -1e9, vmin = 1e9, vmax = -1e9, ss = 0;
    int rise = 0, sink = 0, n1 = 0;
    for (size_t j = 0; j < seg.size(); ++j) {
        const double e = target - seg[j]->d;
        emin = std::min(emin, e); emax = std::max(emax, e);
        vmin = std::min(vmin, seg[j]->vel); vmax = std::max(vmax, seg[j]->vel);
        ss += e * e;
        if (std::fabs(e) < 1.0) ++n1;
        if (seg[j]->intent < 0) ++rise;
        if (seg[j]->intent > 0) ++sink;
        if (j > 0) {
            const double ep = target - seg[j - 1]->d;
            if ((ep < 0) != (e < 0)) zc.push_back(seg[j]->t);
        }
    }
    HoldMetrics h;
    h.period = zc.size() >= 3 ? 2.0 * (zc.back() - zc.front()) / (zc.size() - 1) : NAN;
    h.p2p = emax - emin;
    h.vamp = (vmax - vmin) / 2.0;
    h.risePct = 100.0 * rise / seg.size();
    h.sinkPct = 100.0 * sink / seg.size();
    h.idlePct = 100.0 - h.risePct - h.sinkPct;
    h.rms = std::sqrt(ss / seg.size());
    h.w1 = 100.0 * n1 / seg.size();
    return h;
}

const Schedule kSteps = {{0, 40}, {90, 50}, {180, 40}, {270, 30}, {360, 40}, {450, 40}};
const Schedule kHold = {{0, 40}, {150, 40}};

PlantParams parse(char** a) {
    PlantParams p;
    p.Ks = atof(a[0]); p.Kr = atof(a[1]); p.tauW = atof(a[2]);
    p.L = atof(a[3]); p.aRest = atof(a[4]); p.alpha = atof(a[5]);
    return p;
}

void writeTrace(const RunResult& rr, const char* path) {
    FILE* f = std::fopen(path, "w");
    if (!f) { std::perror(path); return; }
    std::fprintf(f, "t,depth,target,vel,intent_pwm,applied_duty\n");
    for (const auto& s : rr.log)
        std::fprintf(f, "%.2f,%.3f,%.1f,%.3f,%d,%.3f\n", s.t, s.d, s.target, s.vel, s.intent, s.applied);
    std::fclose(f);
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: see header\n"); return 2; }
    const std::string mode = argv[1];
    if (mode == "calib") {
        // Observed, control-1791368554 at 40 cm (settled): T 9.5 s, p2p 7.3 cm, |v| 3.5 cm/s, rise 55 %, sink 30 %
        struct Row { double score; PlantParams p; HoldMetrics h; };
        std::vector<Row> rows;
        for (double Ks : {0.5, 1.0}) for (double Kr : {2.0, 3.0, 4.0}) for (double tw : {1.5, 2.0, 3.0})
        for (double L : {0.1, 0.4}) for (double ar : {0.5, 1.1}) for (double al : {0.0, 0.03}) {
            PlantParams p; p.Ks = Ks; p.Kr = Kr; p.tauW = tw; p.L = L; p.aRest = ar; p.alpha = al;
            const RunResult rr = run(p, kHold, 40.0, 1);
            const HoldMetrics h = holdMetrics(rr, 50.0, 40.0);
            const double T = std::isnan(h.period) ? 1e3 : h.period;
            const double score = std::fabs(T - 9.5) / 9.5 + std::fabs(h.p2p - 7.3) / 7.3 + std::fabs(h.vamp - 3.5) / 3.5 +
                                 std::fabs(h.risePct - 55) / 55 + std::fabs(h.sinkPct - 30) / 30;
            rows.push_back({score, p, h});
        }
        std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.score < b.score; });
        std::printf("score   Ks  Kr tauW   L aRest alpha |  T(s)  p2p  vamp rise%% sink%% idle%%  RMS  <1cm%%\n");
        for (size_t i = 0; i < 12 && i < rows.size(); ++i) {
            const auto& r = rows[i];
            std::printf("%5.2f %4.1f %3.1f %4.1f %3.1f %5.1f %5.2f | %5.1f %4.1f %5.2f %5.0f %5.0f %5.0f %4.2f %5.0f\n",
                        r.score, r.p.Ks, r.p.Kr, r.p.tauW, r.p.L, r.p.aRest, r.p.alpha, r.h.period, r.h.p2p, r.h.vamp,
                        r.h.risePct, r.h.sinkPct, r.h.idlePct, r.h.rms, r.h.w1);
        }
        return 0;
    }
    if (argc < 9) { std::fprintf(stderr, "need Ks Kr tauW L aRest alpha seed\n"); return 2; }
    const PlantParams p = parse(argv + 2);
    const unsigned seed = static_cast<unsigned>(atoi(argv[8]));
    if (mode == "eval") {
        const RunResult rr = run(p, kSteps, 40.0, seed);
        const StepMetrics m = stepMetrics(rr, kSteps, 40.0);
        std::printf("%.4f %.2f %.2f %.3f %.2f %.2f\n", m.rms, m.w1, m.w05, m.over, m.valvePerMin, m.dutyPct);
        return 0;
    }
    if (mode == "hold") {
        const RunResult rr = run(p, kHold, 40.0, seed);
        const HoldMetrics h = holdMetrics(rr, 50.0, 40.0);
        std::printf("%.2f %.2f %.3f %.1f %.1f %.1f %.3f %.1f\n", h.period, h.p2p, h.vamp, h.risePct, h.sinkPct, h.idlePct, h.rms, h.w1);
        if (argc > 9) writeTrace(rr, argv[9]);
        return 0;
    }
    if (mode == "trace") {
        if (argc < 10) { std::fprintf(stderr, "need out.csv\n"); return 2; }
        const RunResult rr = run(p, kSteps, 40.0, seed);
        writeTrace(rr, argv[9]);
        return 0;
    }
    std::fprintf(stderr, "unknown mode\n");
    return 2;
}
