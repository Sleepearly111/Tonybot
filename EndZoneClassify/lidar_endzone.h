/*
 * lidar_endzone.h — 终点区定位
 *
 * 判断机器人是否已到达终点区
 *
 * 策略：
 *   扫描找到置物台上 3 个物体，取中间物体，
 *   当机器人距离中间物体约 80cm 时，判定已到达终点区。
 */

#ifndef LIDAR_ENDZONE_H
#define LIDAR_ENDZONE_H

#include "lidar_common.h"

// 返回 true 表示已到达终点区
bool lidar_isAtEndZone();

#endif
