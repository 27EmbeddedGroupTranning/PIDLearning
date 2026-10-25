#include "tuning.hpp"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <stdexcept>
#include <sstream>

constexpr double pi = 3.14159265358979323846;
double radians(double degrees) { return degrees * pi / 180.0; }
double degrees(double radians_value) { return radians_value * 180.0 / pi; }

// 两个独立转轴：J * 加速度 = 电机力矩 + 外部力矩 - b * 角速度。
// 这是用于理解闭环的简化模型，不包含齿隙、结构弹性和真实电机电流环。
struct Axis {
    double angle = 0, velocity = 0;
    void step(double motor, double disturbance, double dt) {
        const double inertia = 0.035, damping = 0.06;
        velocity += (motor + disturbance - damping * velocity) / inertia * dt;
        angle += velocity * dt;
    }
};

const char* csv_header = "time,yaw_target,pitch_target,yaw,pitch,yaw_u,pitch_u,yaw_p,yaw_i,yaw_d,pitch_p,pitch_i,pitch_d,load\n";

// 持久进程协议，每行：yaw_deg pitch_deg load_Nm steps reset。
// 网页指令经本地服务转发，每 20 ms 推进 10 个 2 ms 控制周期。
// 双环级联：同一 2 ms tick 内先算外环 PID（角度误差 → 速度指令），
// 再算内环 LQR（速度指令 + 实测角速度 → 力矩），最后推进物理状态。
// 状态保存在 C++ 中；目标改变不会重置角度、速度或积分。
int stream() {
    PID yaw_pid(yaw_gains()), pitch_pid(pitch_gains());
    LQR yaw_lqr(yaw_lqr_params()), pitch_lqr(pitch_lqr_params());
    Axis yaw, pitch;
    long long tick = 0;
    const double dt = 0.002;
    double yu = 0, pu = 0;
    std::cout << csv_header << std::flush;
    std::string input;
    while (std::getline(std::cin, input)) {
        std::istringstream command(input);
        double yt, pt, load;
        int steps, reset;
        if (!(command >> yt >> pt >> load >> steps >> reset) ||
            !std::isfinite(yt) || !std::isfinite(pt) || !std::isfinite(load) ||
            std::abs(yt) > 150 || std::abs(pt) > 60 || std::abs(load) > 0.3 || steps < 0 || steps > 10)
            throw std::runtime_error("Invalid stream command");
        if (reset) {
            yaw = Axis(); pitch = Axis(); yaw_pid.reset(); pitch_pid.reset();
            yaw_lqr.reset(); pitch_lqr.reset();
            tick = 0; yu = pu = 0;
        }
        for (int n = 0; n < steps; ++n) {
            // 外环 PID：目标角与实测角之差 → 期望角速度；内环 LQR：期望角速度与
            // 实测角速度（内环状态量，取自 Axis）→ 电机力矩。
            const double yaw_speed_ref = yaw_pid.update(radians(yt), yaw.angle, dt);
            const double pitch_speed_ref = pitch_pid.update(radians(pt), pitch.angle, dt);
            yu = yaw_lqr.update(yaw_speed_ref, yaw.velocity, dt);
            pu = pitch_lqr.update(pitch_speed_ref, pitch.velocity, dt);
            yaw.step(yu, load, dt); pitch.step(pu, load, dt);
            ++tick;
        }
        // 输出当前角度和最近一次实际施加的力矩；控制项采样比角度早至多 2 ms。
        std::cout << std::fixed << std::setprecision(7)
                  << tick * dt << ',' << yt << ',' << pt << ',' << degrees(yaw.angle) << ',' << degrees(pitch.angle)
                  << ',' << yu << ',' << pu << ',' << yaw_pid.p << ',' << yaw_pid.i << ',' << yaw_pid.d
                  << ',' << pitch_pid.p << ',' << pitch_pid.i << ',' << pitch_pid.d << ',' << load << '\n' << std::flush;
    }
    return 0;
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--stream") return stream();
        double yaw_target = 45, pitch_target = 20, load = 0.08;
        bool reverse = false;
        if (argc != 1 && argc != 5) throw std::runtime_error("Usage: gimbal_sim [yaw_deg pitch_deg load_Nm reverse_0_or_1]");
        if (argc == 5) {
            yaw_target = std::stod(argv[1]); pitch_target = std::stod(argv[2]);
            load = std::stod(argv[3]); reverse = std::stoi(argv[4]) != 0;
        }
        if (!std::isfinite(yaw_target) || !std::isfinite(pitch_target) || !std::isfinite(load) ||
            std::abs(yaw_target) > 150 || std::abs(pitch_target) > 60 || std::abs(load) > 0.3)
            throw std::runtime_error("Inputs out of range");
        PID yaw_pid(yaw_gains()), pitch_pid(pitch_gains());
        LQR yaw_lqr(yaw_lqr_params()), pitch_lqr(pitch_lqr_params());
        Axis yaw, pitch;
        const double dt = 0.002; // 固定 500 Hz 控制周期，不受浏览器播放速度影响。
        std::cout << csv_header;
        std::cout << std::fixed << std::setprecision(7);
        for (int n = 0; n <= 7500; ++n) {
            double t = n * dt;
            const double direction = reverse && t >= 8 ? -1 : 1;
            const double yt = t >= 1 ? radians(yaw_target) * direction : 0;
            const double pt = t >= 1 ? radians(pitch_target) * direction : 0;
            const double external = t >= 6 ? load : 0;
            // 外环 PID 输出期望角速度，内环 LQR 据此与实测角速度解算力矩。
            const double yaw_speed_ref = yaw_pid.update(yt, yaw.angle, dt);
            const double pitch_speed_ref = pitch_pid.update(pt, pitch.angle, dt);
            const double yu = yaw_lqr.update(yaw_speed_ref, yaw.velocity, dt);
            const double pu = pitch_lqr.update(pitch_speed_ref, pitch.velocity, dt);
            // 记录当前采样时刻的角度和据此计算的力矩，再更新下一时刻的物理状态。
            if (n % 10 == 0) {
                std::cout << t << ',' << degrees(yt) << ',' << degrees(pt) << ','
                          << degrees(yaw.angle) << ',' << degrees(pitch.angle) << ',' << yu << ',' << pu << ','
                          << yaw_pid.p << ',' << yaw_pid.i << ',' << yaw_pid.d << ','
                          << pitch_pid.p << ',' << pitch_pid.i << ',' << pitch_pid.d << ',' << external << '\n';
            }
            yaw.step(yu, external, dt); pitch.step(pu, external, dt);
        }
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
