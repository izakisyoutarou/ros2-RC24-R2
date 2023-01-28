#pragma once
#include <float.h>
#include <math.h>

namespace cfg{

namespace robot{
    constexpr float move_pos_limit = FLT_MAX;
    constexpr float move_vel_limit = 90 * 3.1415926535897932384626433832795;  //ロボットの並進速度の大きさの制限(最大値)[m/s]
    constexpr float move_acc_limit = 60 * 3.1415926535897932384626433832795;  //[m/s^2]
    constexpr float move_dec_limit = move_acc_limit;  /// [m/s^2]
    constexpr float rotate_vel_limit = 3.1415926535897932384626433832795 * 180 / 180;
}
}