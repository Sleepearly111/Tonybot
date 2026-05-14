#ifndef CIRCLEPILLAR_HPP
#define CIRCLEPILLAR_HPP

#include <Arduino.h>

#define CIRCLE_LEFT  0
#define CIRCLE_RIGHT 1

void circlePillar_init();

/**
 * @brief 绕柱主更新函数（需在主循环中高频调用）
 * @param color_reg   颜色寄存器（TRACK_COLOR_RED 等）
 * @param direction   绕柱方向 CIRCLE_LEFT 或 CIRCLE_RIGHT
 */
void circlePillar_update(uint8_t color_reg, uint8_t direction);

// 返回 true 表示绕柱已完成
int circlePillar_getSteps();
bool circlePillar_isDone();

#endif
