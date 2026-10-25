#include "pid.hpp"
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
        std::cout << "PID tests passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
