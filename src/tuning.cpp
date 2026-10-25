#include "tuning.hpp"

// 实验主入口：修改这里，保存，然后在网页点击“重新编译 C++”。
// 角度单位 rad，力矩单位 N m。先试 Kp=0.014, Ki=0.010, Kd=0.0038，再逐项调整。
Gains yaw_gains() {
    return {0.06, 0.0, 0.1, 0.65, 0.025, true};
}
Gains pitch_gains() {
    return {0.06, 0.0, 0.1, 0.65, 0.025, true};
}
