#include "tuning.hpp"

// 实验主入口：修改这里，保存，然后在网页点击“编译并运行”。
// 角度单位 rad，力矩单位 N m。先试 Kp=0.8, Ki=0, Kd=0，再逐项调整。
Gains yaw_gains() {
    return {0.8, 0.45, 0.22, 0.65, 0.025, true};
}
Gains pitch_gains() {
    return {0.8, 0.45, 0.22, 0.65, 0.025, true};
}
