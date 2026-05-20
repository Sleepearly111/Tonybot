#ifndef HEADTRACKER_HPP
#define HEADTRACKER_HPP

#include <Arduino.h>

// 颜色模式命令码，与 ESP32-S3 端保持一致：0x01=红色，0x02=蓝色
#define TRACK_COLOR_RED   0x01
#define TRACK_COLOR_BLUE  0x02

void tracker_init();
void tracker_set_color(uint8_t color_mode);  // 发送颜色切换命令给 ESP32
void tracker_update();                        // 仅读取数据并控制舵机，不发送切换命令
uint8_t get_head_angle();
// 在现有声明后添加
void headTracker_resetToCenter();
uint8_t get_last_x();
uint8_t get_width();
// 非阻塞稳定判断（只读，不做任何控制）
bool isAngleStable(uint8_t &stable_angle);
void resetStableCheck();   // 重置状态机，准备下一次判断
#endif