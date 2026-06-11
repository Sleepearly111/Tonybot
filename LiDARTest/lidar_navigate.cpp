/*
 * lidar_navigate.cpp — 识别区导航
 *
 * TODO (B): 以 3 个物体为信标导航到置物台前
 *
 * 3 个物体定位思路：
 *   1. 扫描前方 ±60°（或全周），找到距离 500~2000mm 的有效点
 *   2. 连续的有效点组成"物体"（相邻角度间距 < 8°，距离差 < 300mm）
 *   3. 找到 3 个物体 → 计算它们的中心角度 → 作为导航方向
 *   4. 修正：机器人当前朝向 vs 物体中心方向 → 转弯量
 *   5. 前进：靠近到目标距离（标记区在置物台前 200mm）
 *
 * 注意：
 *   - LiDAR 安装偏移：雷达 90° = 机器人前方 0°
 *   - 置物台物体前方 200mm 是标记区，不要撞到台子
 */

#include "lidar_navigate.h"

NavCmd lidar_nav_toPlatform() {
    NavCmd cmd = {0, 0, false};
    // TODO: 实现导航逻辑
    return cmd;
}

int lidar_findObjects(int* outAngles) {
    // TODO: 在 g_map[] 中找 3 个物体，返回找到的个数
    return 0;
}
