/*
 * lidar_navigate.h — 识别区导航（两阶段）
 *
 * 阶段1 转正：转身对准中间物体
 * 阶段2 逼近：直走到中间物体前方 80cm（终点区）
 */

#ifndef LIDAR_NAVIGATE_H
#define LIDAR_NAVIGATE_H

#include "lidar_common.h"

// 导航阶段
enum NavPhase {
    NAV_FACE_OBJ,     // 阶段1：转正对准中间物体
    NAV_APPROACH,     // 阶段2：逼近
    NAV_ARRIVED       // 到达终点区
};

// 重置导航状态（每次导航开始时调用）
void lidar_nav_reset();

// 计算下一步导航指令
NavCmd lidar_nav_toPlatform();

// 返回当前阶段（给串口显示用）
NavPhase lidar_nav_phase();

// 在前方找 3 个物体，返回角度
int lidar_findObjects(int* outAngles);

#endif
