/*
 * lidar_common.h — LiDAR 共享数据 + 初始化
 * 所有模块都 include 这个头文件
 */

#ifndef LIDAR_COMMON_H
#define LIDAR_COMMON_H

#include "RPLidar.h"

// === 引脚 ===
#define PIN_LIDAR_RX    32
#define PIN_LIDAR_TX    33
#define PIN_LIDAR_MOTOR 13

// === 扫描数据（1° 分辨率，360 个点）===
extern float   g_map[360];        // 距离 (mm)，0 = 无效
extern uint8_t g_qual[360];       // 信号质量 (0~255)
extern bool    g_scanReady;       // 刚完成一圈 = true

// === LiDAR 对象（全局共享）===
extern RPLidar lidar;

// === 导航指令（navigate1 和 navigate2 共用）===
struct NavCmd {
    int  turnAngle;   // ° 正=左转 负=右转 0=直行
    int  forwardMm;   // mm
    bool arrived;     // true=到达
    bool smallTurn;   // true=小幅度转弯
};

// === 函数 ===
void lidar_init();
void lidar_update();
void lidar_streamScan();
int  lidar_findObjects(int* outAngles);  // 在 lidar_navigate.cpp 里

#endif
