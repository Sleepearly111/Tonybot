#ifndef ROBOT_TRACKING_H
#define ROBOT_TRACKING_H

#include <Arduino.h>

// 初始化追踪模块（重置头部角度和状态）
void init_tracking();

// 核心追踪接口：把 S3 传来的坐标喂给它
void execute_pid_tracking(uint16_t x, uint8_t y);

#endif