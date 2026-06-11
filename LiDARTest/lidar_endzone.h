/*
 * lidar_endzone.h — 终点区定位
 *
 * 判断机器人是否已到达终点区（通道二之后的大区）
 *
 * 思路：
 *   穿过通道二（500×500）后，前方突然变宽（2100mm 宽场地）。
 *   此时雷达扫到的是空旷场地 + 远方置物台上的 3 个物体。
 *   当正前方距离 > 某个阈值，且 ±60° 视野内检测到 3 个
 *   在 500~2000mm 范围内的物体 → 判定已到终点区。
 */

#ifndef LIDAR_ENDZONE_H
#define LIDAR_ENDZONE_H

#include "lidar_common.h"

// 返回 true 表示已到达终点区
bool lidar_isAtEndZone();

#endif
