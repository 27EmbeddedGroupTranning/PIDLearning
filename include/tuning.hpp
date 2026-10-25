#pragma once
#include "pid.hpp"
#include "lqr.hpp"
Gains yaw_gains();
Gains pitch_gains();
LQRParams yaw_lqr_params();
LQRParams pitch_lqr_params();
