/*
 * LiDARTest.ino — LiDAR 模块化测试程序 (同学B Arduino IDE)
 *
 * 接线：GPIO32=RX, GPIO33=TX, GPIO13=MOTOR, 5V+GND
 * PC可视化: python tools/lidar_viewer.py <COM口>
 *
 * 模块:
 *   lidar_common     — 雷达初始化 + 数据采集 (不需要改)
 *   lidar_endzone    — 终点区检测 (TODO: B)
 *   lidar_navigate   — 识别区导航 (TODO: B)
 *   lidar_classify   — 物体形状识别 (已完成, 待实测)
 */

#include "lidar_common.h"
#include "lidar_endzone.h"
#include "lidar_navigate.h"
#include "lidar_classify.h"

// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n====== LiDAR 模块化测试 ======");

    lidar_init();
    Serial.println("OK\n");
}

// ============================================================
void loop() {
    lidar_update();       // 每帧吃一个扫描点

    if (!g_scanReady) return;
    g_scanReady = false;

    // ---- 流式输出（PC 可视化） ----
    lidar_streamScan();

    // ---- 终点区检测 (TODO) ----
    // if (lidar_isAtEndZone()) {
    //     Serial.println("[EndZone] 已到达终点区");
    // }

    // ---- 识别区导航 (TODO) ----
    // NavCmd cmd = lidar_nav_toPlatform();
    // Serial.printf("[Nav] turn=%d° fwd=%dmm arrived=%d\n",
    //               cmd.turnAngle, cmd.forwardMm, cmd.arrived);

    // ---- 物体识别 (可用, 待实测) ----
    // int objs[3], n = lidar_findObjects(objs);
    // for (int i = 0; i < n; i++) {
    //     int shape = classifyObject(objs[i]);
    //     Serial.printf("[Classify] %d° → %d\n", objs[i], shape);
    // }
}
