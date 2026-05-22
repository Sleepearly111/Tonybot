#ifndef OBJECTFOLLOWER_HPP
#define OBJECTFOLLOWER_HPP

#include <Arduino.h>

void objectFollower_init();

/**
 * @brief 对象跟随主更新函数（需在主循环中高频调用）
 * @param color_reg  跟踪的颜色寄存器（TRACK_COLOR_RED 等）
 * @param deadband   对准死区（度），默认5°
 */
void objectFollower_update(uint8_t color_reg, int deadband = 5);

#endif
