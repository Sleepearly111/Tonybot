#ifndef ARENA_H
#define ARENA_H

// ——————————————————————————————————————————————
// CRAIC 2026 场地尺寸 (mm)
// ——————————————————————————————————————————————

// 场地总尺寸
#define ARENA_TOTAL_DEPTH       4400   // 600+500+1300+500+600+500+400
#define ARENA_TOTAL_WIDTH       2100

// 区域深度
#define LARGE_START_DEPTH        600   // 大起点区
#define CHANNEL1_DEPTH           500   // 通道一
#define OBSTACLE_ZONE_DEPTH     1300   // 避障区
#define CHANNEL2_DEPTH           500   // 通道二
#define LARGE_END_DEPTH          600   // 大终点区
#define GND_GAP_DEPTH            500   // 空区（大终点区到置物台之间）
#define PLATFORM_DEPTH           400   // 置物台
#define RECOG_ZONE_DEPTH        2000   // 通道二+大终点区+空区+置物台

// 窄区宽度 (在 2100 内居中)
#define NARROW_ZONE_WIDTH        500
#define NARROW_ZONE_OFFSET       800   // (2100-500)/2

// 起点/终点
#define SMALL_BOX_SIZE           200   // 小起点/小终点

// 置物台
#define PLATFORM_WIDTH          1800
#define PLATFORM_HEIGHT          300   // LiDAR 平面 > 此值，扫不到台面

// 物体
#define OBJECT_SPACING           300   // 物体中心间距
#define OBJECT_MIN_DIAMETER      200
#define OBJECT_MAX_DIAMETER      300

// 标记区
#define MARKER_SIZE              150   // 正方形边长
#define MARKER_OFFSET            200   // 距置物台前缘距离

// 柱子
#define PILLAR_DIAMETER          300
#define PILLAR_HEIGHT            300   // LiDAR 平面 > 此值，扫不到
#define PILLAR_RADIUS            150

// 通道
#define CHANNEL_WIDTH            500
#define CHANNEL_HEIGHT           500
#define CHANNEL_LENGTH           500

// 导航参数
#define COLLISION_DIST_MM        200   // 前方碰撞距离阈值
#define ALIGN_TOLERANCE_DEG        5   // 星座居中对齐容差 (度)
#define OBJECT_DETECT_RANGE     4000   // 物体检测最大距离

// 计时 (ms)
#define TASK1_TIMEOUT_MS     300000    // 5 min
#define TASK23_TIMEOUT_MS    300000    // 5 min

#endif
