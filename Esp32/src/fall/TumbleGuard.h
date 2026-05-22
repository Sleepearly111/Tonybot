#ifndef TUMBLE_GUARD_H
#define TUMBLE_GUARD_H

// 基础依赖（必须）
#include "Arduino.h"
// 项目核心依赖（主程序已包含）
#include "../base_config.h"
#include "../LobotServoController.h"
#include "../Hiwonder.hpp"

// ============== 外部对象声明（主程序已初始化，这里仅引用）==============
extern IMU imu;
extern LobotServoController Controller;


// ============== 姿态变量声明 ==============
extern float radianX;
extern float radianY;

// ============== 核心功能函数 ==============
void tumble();

#endif