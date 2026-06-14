#ifndef PILLARROUTE_HPP
#define PILLARROUTE_HPP

#include <Arduino.h>
#include "HeadTracker.hpp"

// 绕柱方向
#define ROUTE_LEFT  0
#define ROUTE_RIGHT 1

// 四种绕柱流程
// ROUTE_BLUE_RIGHT  → 先蓝柱+右绕 → 后红柱+左绕
// ROUTE_BLUE_LEFT   → 先蓝柱+左绕 → 后红柱+右绕
// ROUTE_RED_RIGHT   → 先红柱+右绕 → 后蓝柱+左绕
// ROUTE_RED_LEFT    → 先红柱+左绕 → 后蓝柱+右绕
enum PillarRoute {
    ROUTE_BLUE_RIGHT,
    ROUTE_BLUE_LEFT,
    ROUTE_RED_RIGHT,
    ROUTE_RED_LEFT,
};

// 启动指定流程（主程序一句话调用）
void pillarRoute_start(PillarRoute route);

// 当前阶段的颜色和绕向
uint8_t pillarRoute_color();
uint8_t pillarRoute_direction();

// 当前阶段完成时调用，切到下一阶段。返回 true = 全部完成
bool pillarRoute_next();

#endif
