#include "tuning.hpp"

// 实验主入口：修改这里，保存，然后在网页点击“重新编译 C++”。
// 角度单位 rad，力矩单位 N m。双环下外环输出为速度指令：
// max_output 的量纲是速度上限（rad/s），Kp 为每 rad 误差发出的速度指令。
// 双环结构内环 LQR 已提供速度阻尼，外环 Kp 可比单环时代更大。
Gains yaw_gains() {
    return {16.0, 0.9, 0.3, 5.0, 0.025, true}; ;
}
Gains pitch_gains() {
    return {16.0, 0.9, 0.3, 5.0, 0.025, true}; ;
}

// 内环 LQR 配置：外环 PID 的输出（限幅值在此语义下为速度指令上限 rad/s）
// 作为速度参考，内环按模型 J·v̇ = u − b·v 解算力矩。Q/R 调节跟踪紧致度与
// 力矩平缓度的折中；max_output 是内环力矩限幅（N·m），放开它可直接提高
// 加速度上限（注意与 lab.py --test 的执行器断言联动）。enabled=false 时
// 外环输出直接作为力矩下发，即回到原单环 PID 行为。
LQRParams yaw_lqr_params() {
    return {0.035, 0.06, 1.0, 1.0, 1.0, true};
}
LQRParams pitch_lqr_params() {
    return {0.035, 0.06, 1.0, 1.0, 1.0, true};
}
