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

// === 函数 ===
void lidar_init();                // 初始化雷达 + 电机
void lidar_update();              // 每帧吃一个扫描点，填 g_map[]
void lidar_streamScan();          // 输出当前圈到串口 (SCAN...END)

#endif
