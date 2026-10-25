#include "pid.hpp"
#include "lqr.hpp"
#include <iostream>
#include <stdexcept>
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int main() {
    try {
        PID proportional({2, 0, 0, 1, 0.025, true});
        check(proportional.update(10, 0, 0.002) == 1, "positive saturation");
        check(proportional.update(-10, 0, 0.002) == -1, "negative saturation");
        check(proportional.update(1, 0, 0) == 0, "invalid dt");
        PID derivative({0, 0, 1, 10, 0.025, true});
        derivative.update(0, 0, 0.002);
        check(derivative.update(100, 0, 0.002) == 0, "derivative kick");
        check(derivative.update(100, 0.1, 0.002) < 0, "derivative opposes motion");
        PID protected_pid({1, 1, 0, 1, 0.025, true});
        PID unprotected_pid({1, 1, 0, 1, 0.025, false});
        for (int n = 0; n < 1000; ++n) {
            protected_pid.update(10, 0, 0.002); unprotected_pid.update(10, 0, 0.002);
        }
        check(protected_pid.update(-0.1, 0, 0.002) < 0, "anti-windup must allow reversal");
        check(unprotected_pid.update(-0.1, 0, 0.002) > 0, "windup demonstration");
        protected_pid.reset();
        check(std::abs(protected_pid.update(0, 0, 0.002)) < 1e-12, "reset");

        // 内环 LQR：增益求解、速度参考跟踪、前馈、限幅与旁路。
        const double j = 0.035, b = 0.06, dt = 0.002;
        LQR lqr({j, b, 1.0, 1.0, 0.65, true});
        check(std::isfinite(lqr.gain()) && lqr.gain() > 0, "lqr gain finite positive");
        double v = 0;
        for (int n = 0; n < 7500; ++n) {  // 15 s 闭环仿真：速度环应收敛到参考
            const double u = lqr.update(1.0, v, dt);
            check(std::abs(u) <= 0.650001, "lqr output within limits");
            v += (u - b * v) / j * dt;
        }
        check(std::abs(v - 1.0) < 0.01, "lqr tracks speed reference");
        check(std::abs(lqr.update(1.0, 1.0, dt) - b) < 1e-9, "feedforward cancels damping at steady state");
        check(lqr.update(1.0, 0.0, dt) == 0.65, "lqr saturation");
        check(lqr.update(0.0, 5.0, dt) == -0.65, "lqr negative saturation");
        LQR bypass({j, b, 1.0, 1.0, 0.65, false});
        check(bypass.update(0.5, 0.1, dt) == 0.5, "lqr bypass keeps pid output as torque");

        std::cout << "PID tests passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
