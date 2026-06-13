/*
 * lidar_endzone.cpp — 终点区定位
 *
 * 策略：
 *   扫描找到置物台上 3 个物体 → 取中间物体 →
 *   当机器人距离中间物体约 80cm 时，判定已到达终点区。
 *
 * 复用了 lidar_navigate 里的 lidar_findObjects()。
 */

#include "lidar_endzone.h"
#include "lidar_navigate.h"

// 目标距离及容忍范围
#define TARGET_DIST_MM    800
#define TOLERANCE_MM      200   // ±200mm 容忍

bool lidar_isAtEndZone() {

    // ---- 1. 找到物体 ----
    int objAngles[3];
    int nFound = lidar_findObjects(objAngles);

    if (nFound < 1) {
        return false;  // 没找到物体 → 肯定不在终点区
    }

    // ---- 2. 确定中间物体 ----
    int middleAngle;
    if (nFound == 1) {
        middleAngle = objAngles[0];
    } else if (nFound == 2) {
        // 两个物体，取离机器人前方 (90°) 更近的
        auto diff = [](int a, int b) {
            int d = abs(a - b);
            return (d > 180) ? 360 - d : d;
        };
        int d0 = diff(objAngles[0], 90);
        int d1 = diff(objAngles[1], 90);
        middleAngle = (d0 < d1) ? objAngles[0] : objAngles[1];
    } else {
        // 三个物体，取角度居中的 (索引 1，已排序)
        middleAngle = objAngles[1];
    }

    // ---- 3. 检查距离 ----
    float dist = g_map[middleAngle % 360];
    if (dist <= 0) {
        return false;
    }

    // 距离在 [800-200, 800+200] = [600, 1000] mm 范围内就算到达
    if (dist >= (TARGET_DIST_MM - TOLERANCE_MM) &&
        dist <= (TARGET_DIST_MM + TOLERANCE_MM)) {
        return true;
    }

    return false;
}
