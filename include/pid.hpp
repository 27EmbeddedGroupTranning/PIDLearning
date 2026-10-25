#pragma once
#include <algorithm>
#include <cmath>

struct Gains {
    double kp, ki, kd;
    double max_output;      // 电机力矩上限，N m
    double derivative_tau;  // 测量微分的一阶低通时间常数，s
    bool anti_windup;
};

class PID {
public:
    explicit PID(Gains gains) : g(gains) {}
    void reset() { integral = derivative = previous_measurement = p = i = d = 0; first = true; }
    double update(double target, double measurement, double dt) {
        if (!(dt > 0) || !std::isfinite(dt)) return 0;
        const double error = target - measurement;
        // 对测量值微分：目标阶跃不会直接导致 D 项冲击。
        const double raw = first ? 0 : -(measurement - previous_measurement) / dt;
        derivative += dt / (g.derivative_tau + dt) * (raw - derivative);
        previous_measurement = measurement;
        first = false;
        const double candidate = integral + error * dt;
        const double trial = g.kp * error + g.ki * candidate + g.kd * derivative;
        // 条件积分：饱和且误差将继续推向饱和方向时，冻结积分。
        const bool pushes_saturation = (trial > g.max_output && error > 0) ||
                                       (trial < -g.max_output && error < 0);
        if (g.ki != 0 && (!g.anti_windup || !pushes_saturation)) integral = candidate;
        p = g.kp * error;
        i = g.ki * integral;
        d = g.kd * derivative;
        return std::max(-g.max_output, std::min(g.max_output, p + i + d));
    }
    double p = 0, i = 0, d = 0;
private:
    Gains g;
    double integral = 0, derivative = 0, previous_measurement = 0;
    bool first = true;
};
