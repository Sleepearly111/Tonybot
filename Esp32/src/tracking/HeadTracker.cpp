#include "HeadTracker.hpp"
#include "../base_config.h"
#include "../Servo.h"
#include "../hw_esp32cam_ctl.h"
#include "../LobotServoController.h"
#include <Wire.h>

extern LobotServoController Controller;
extern Servo sonarServo;

static const uint8_t left  = 90;        // 死区阈值
static const uint8_t right = 150;
static uint8_t dev = 1;
static uint8_t angle = 90;
static uint8_t last_width = 0;
static uint8_t last_x = 0;



// 实现函数
uint8_t get_last_x() {
    return last_x;
}

uint8_t get_width() {
    return last_width;
}

void tracker_init() {
  sonarServo.attach(IO_Servo);
  sonarServo.write(90);
  delay(200);
  tracker_set_color(TRACK_COLOR_RED); // 上电同步，默认红色模式
}

// 发送颜色模式切换命令给 ESP32（仅发送，不读取数据）
void tracker_set_color(uint8_t color_mode) {
  Wire.beginTransmission(ESP32CAM_ADDR);
  Wire.write(color_mode);
  Wire.endTransmission();
}

// 仅读取数据并控制舵机，不发送颜色切换命令
void tracker_update() {
  static unsigned long lastPrint = 0;

  Controller.receiveHandle();

  Wire.requestFrom(ESP32CAM_ADDR, 4);
  if (Wire.available() != 4) {
    if (millis() - lastPrint >= 500) {
      Serial.print("angle:"); Serial.print(angle);
      Serial.println("\twidth:noI2C");
      lastPrint = millis();
    }
    return;
  }

  uint8_t data[4];
  for (int i = 0; i < 4; i++) data[i] = Wire.read();

  uint8_t x = data[0];
  last_x = x;
  last_width = data[2];

  if (data[2] == 0) {
    // 没检测到颜色，只输出不控舵机
    if (millis() - lastPrint >= 500) {
      Serial.print("angle:"); Serial.print(angle);
      Serial.println("\twidth:0");
      lastPrint = millis();
    }
    return;
  }

  if (x > right) {
    dev = (x - right) * 0.04;
    angle = (angle - dev) < 0 ? 0 : (angle - dev);
  } else if (x < left) {
    dev = (left - x) * 0.04;
    angle = (angle + dev) > 180 ? 180 : (angle + dev);
  }
  sonarServo.write(angle);

  if (millis() - lastPrint >= 500) {
    Serial.print("angle:"); Serial.print(angle);
    Serial.print("\twidth:"); Serial.println(data[2]);
    lastPrint = millis();
  }
}

uint8_t get_head_angle() {
  return angle;
}

// 稳定判断状态
static uint8_t  _stable_last = 0;
static int      _stable_count = 0;

/**
 * @brief 只读取角度，连续三次读取到的值完全不变则返回 true
 * @param stable_angle 输出稳定角度值
 * @return true: 已稳定, false: 未稳定或正在计数
 */
bool isAngleStable(uint8_t &stable_angle) {
    uint8_t cur = get_head_angle();
    if (cur == _stable_last) {
        _stable_count++;
    } else {
        _stable_last = cur;
        _stable_count = 1;
    }
    if (_stable_count >= 3) {
        stable_angle = cur;
        return true;
    }
    return false;
}

void resetStableCheck() {
    _stable_last = get_head_angle();
    _stable_count = 1;
}

void headTracker_resetToCenter() {
    angle = 90;
    sonarServo.write(90);
}
