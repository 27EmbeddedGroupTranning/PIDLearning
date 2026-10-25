#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

// 内环 LQR 控制器（速度环）。
//
// 双环结构：外环 PID 输出期望角速度 speed_ref（rad/s），内环 LQR 依据
// 被控对象的一阶速度模型输出电机力矩 u（N m）：
//
//     对象模型：J * dv/dt = u - b * v          （与 main.cpp 中 Axis 一致）
//     状态空间：x = v（角速度），A = -b/J，B = 1/J
//     二次代价：J = Σ (Q * x² + R * u²)        Q、R 为权重矩阵（本对象为单状态 SISO）
//     最优解：  u = b * speed_ref - K * (v - speed_ref)
//               其中 b * speed_ref 为前馈（稳态时恰好抵消阻尼），
//               K 由离散代数 Riccati 方程（DARE）迭代求解。
//
// 状态量获取：v 直接取自仿真轴状态（main.cpp 的 Axis.velocity）；
// 实物上对应编码器测得的角度差分或陀螺仪角速度。
//
// enabled = false 时进入旁路：外环输出按原单环语义直接作为力矩返回，
// 与历史 PID 行为完全兼容。
struct LQRParams {
    double inertia;     // 转动惯量 J，kg·m²，须与物理模型一致
    double damping;     // 粘性阻尼 b，N·m·s/rad
    double q_velocity;  // 状态权重 Q：越大速度跟踪越紧
    double r_torque;    // 输入权重 R：越大力矩越平缓
    double max_output;  // 输出力矩限幅，N m
    bool enabled;       // false = 旁路，保持原单环 PID 行为
};

class LQR {
public:
    explicit LQR(LQRParams params) : g(params) {
        if (!(g.inertia > 0) || !(g.r_torque > 0) || g.q_velocity < 0 ||
            !std::isfinite(g.q_velocity) || !std::isfinite(g.r_torque))
            throw std::runtime_error("Invalid LQR parameters");
        K = solve_gain();
    }
    void reset() {}  // 纯状态反馈无内部记忆；保留接口与 PID 对齐
    double gain() const { return K; }

    // speed_ref：外环 PID 输出的期望角速度（rad/s）；velocity：实测角速度。
    // dt 为外环传入的控制周期；一阶增益型解算不依赖步长，仅为接口一致性保留。
    double update(double speed_ref, double velocity, double dt) {
        (void)dt;
        if (!std::isfinite(speed_ref) || !std::isfinite(velocity)) return 0;
        if (!g.enabled) return speed_ref;  // 旁路：外环输出直接作为力矩（原行为）
        const double u = (g.damping + K) * speed_ref - K * velocity;
        return std::max(-g.max_output, std::min(g.max_output, u));
    }

private:
    // 前向欧拉离散化（Ts 与 main.cpp 的 2 ms 控制周期一致）后迭代求解 DARE：
    //   P ← Q + Ad·P·Ad − Ad·P·Bd·(R + Bd·P·Bd)⁻¹·Bd·P·Ad，K = Ad·Bd·P / (R + Bd²·P)
    double solve_gain() const {
        const double ts = 0.002;
        const double ad = 1.0 + (-g.damping / g.inertia) * ts;
        const double bd = ts / g.inertia;
        double p = g.q_velocity;
        for (int n = 0; n < 20000; ++n) {
            const double denom = g.r_torque + bd * bd * p;
            const double next = g.q_velocity + ad * ad * p -
                                ad * ad * bd * bd * p * p / denom;
            if (!std::isfinite(next)) throw std::runtime_error("LQR Riccati diverged");
            const bool converged = std::abs(next - p) < 1e-15 * std::max(1.0, std::abs(next));
            p = next;
            if (converged) break;
        }
        return ad * bd * p / (g.r_torque + bd * bd * p);
    }

    LQRParams g;
    double K = 0;
};
