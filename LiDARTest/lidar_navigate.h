/*
 * lidar_navigate.h — 识别区导航
 *
 * 从终点区导航到置物台前 200mm（标记区位置）
 *
 * 识别区 = 通道二(500) + 终点区(600) + 空区(500) + 置物台(400) = 2000mm 深
 * 置物台高 300mm，台上 3 个物体间距 300mm，距地面 300mm+
 * 标记区 = 3 个 150×150 框，距置物台 200mm
 *
 * 核心思路：置物台上的 3 个物体从头到尾在 LiDAR 视野中，
 * 用它们作为信标导航。
 */

#ifndef LIDAR_NAVIGATE_H
#define LIDAR_NAVIGATE_H

#include "lidar_common.h"

// === 导航结果 ===
struct NavCmd {
    int  turnAngle;   // 需要转的角度 (°)，正=左转 负=右转 0=直行
    int  forwardMm;   // 建议前进距离 (mm)，0=到位
    bool arrived;     // true = 已到达标记区
};

// 计算下一步导航指令（以置物台上 3 物体为信标）
NavCmd lidar_nav_toPlatform();

// 返回置物台区域中 3 个物体的角度（-1 = 没找到该物体）
// outAngles[] 必须 >= 3 个 int
int  lidar_findObjects(int* outAngles);

#endif
