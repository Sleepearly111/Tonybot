// ============================================================
// 全局对象定义（供所有模块链接）
// ============================================================
#include "LobotServoController.h"
#include "voice/HWSensor.h"
#include "Servo.h"
#include "Hiwonder.hpp"

LobotServoController Controller(Serial2);
HWSensor hwsensor;
Servo sonarServo;
IMU imu;
