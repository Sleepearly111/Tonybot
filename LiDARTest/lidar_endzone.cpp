/*
 * lidar_endzone.cpp — 终点区定位
 *
 * TODO (B): 用 g_map[] 判断是否到达终点区
 *
 * 可用数据：
 *   g_map[i]   — 角度 i 的距离 (mm)，0 = 无效
 *   g_qual[i]  — 信号质量
 *
 * 提示：
 *   1. 通道二是 500×500 窄口，穿过后前方突然开阔
 *   2. 终点区深 600mm，宽 2100mm
 *   3. 3 个物体在正前方 2~3 米的置物台上，约束：
 *      - LiDAR 只看到置物台上的物体（柱子/台面在地上 300mm 以下看不到）
 *      - 物体间距 300mm，从左到右排列
 */

#include "lidar_endzone.h"

bool lidar_isAtEndZone() {
    // TODO: 实现终点区检测逻辑
    return false;
}
