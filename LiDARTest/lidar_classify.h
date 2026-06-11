/*
 * lidar_classify.h — 物体形状识别
 *
 * 用 LiDAR 数据区分球体 / 正方体 / 圆柱体
 *
 * 原理（同学A 已验证）：
 *   正方体 → 平面反馈（距离标准差小）+ 边缘陡峭
 *   圆柱体 → 圆弧面，角宽度约为 2·arcsin(150/(D+150))
 *   球体   → 圆弧面（截面半径小），角宽度明显小于圆柱体
 */

#ifndef LIDAR_CLASSIFY_H
#define LIDAR_CLASSIFY_H

#include "lidar_common.h"

struct ShapeInfo {
    int   centerAngle;    // 物体中心角度
    int   angularSpan;    // 物体占据的度数
    float minDist;        // 最近距离 (mm)
    float maxDist;        // 最远距离 (mm)
    float distStddev;     // 距离标准差
    bool  sharpEdges;     // 边缘是否陡峭
};

// 识别指定角度处物体的形状
// 返回值: 0=未知 1=球体 2=正方体 3=圆柱体
int classifyObject(int angleDeg, ShapeInfo* info = nullptr);

#endif
