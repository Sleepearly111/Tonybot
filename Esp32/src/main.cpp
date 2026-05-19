#include <Arduino.h>
#include "LidarDriver.h"

// ============================================================
// A1M8 (RPLIDAR) 引脚定义
// ============================================================
#define PIN_LIDAR_RX     32   // A1M8 TX → ESP32 RX (GPIO32, 非 strapping)
#define PIN_LIDAR_TX     33   // A1M8 RX → ESP32 TX (GPIO33, 非 strapping)
#define PIN_LIDAR_MOTOR  13   // A1M8 MOTOCTRL

LidarDriver lidar;

// 统计用
static unsigned long lastPrintMs = 0;
static uint32_t scanCount = 0;

// ============================================================
// setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println("  A1M8 激光雷达通信测试");
    Serial.println("  A1M8 TX→GPIO32, RX→GPIO33, MOTO→GPIO13");
    Serial.println("========================================");

    // 初始化激光雷达
    if (!lidar.begin(PIN_LIDAR_RX, PIN_LIDAR_TX, PIN_LIDAR_MOTOR)) {
        Serial.println("[FATAL] 雷达初始化失败！请检查接线和供电。");
        while (1) {
            delay(1000);
            Serial.println("等待重启...");
        }
    }

    Serial.println("[OK] 雷达已就绪，等待扫描数据...");
    lastPrintMs = millis();
}

// ============================================================
// loop()
// ============================================================
void loop() {
    // 1. 处理雷达数据点
    lidar.update();

    // 2. 检测到完整一圈扫描
    if (lidar.isScanComplete()) {
        scanCount++;
        lidar.resetScanFlag();
    }

    // 3. 每 2 秒输出一次摘要
    if (millis() - lastPrintMs > 2000) {
        lastPrintMs = millis();

        Serial.printf("\n--- 扫描 #%u ---\n", scanCount);
        lidar.printSummary();

        // 检测前方 30° 扇形内是否有障碍物
        bool frontClear = lidar.isSectorClear(0, 30, 800);
        Serial.printf("  前方30° 空旷? %s\n", frontClear ? "YES" : "NO");

        // 找最空旷的方向（正负90°前方半圆内）
        int best = lidar.findFarthest(315, 45);  // -45° ~ +45°
        if (best >= 0) {
            Serial.printf("  前方最空旷方向: %d°  (%.0fmm)\n",
                          best, lidar.distanceAt(best));
        }

        // === 形状识别测试 ===
        // 在前方 180° 范围内找最近的物体
        int objAngle = lidar.findObjectInSector(270, 90, 1500);
        if (objAngle >= 0) {
            LidarDriver::ShapeInfo shapeInfo;
            int shape = lidar.classifyObjectAt(objAngle, &shapeInfo);
            const char* shapeName = "未知";
            if (shape == 1) shapeName = "球体";
            else if (shape == 2) shapeName = "正方体";
            else if (shape == 3) shapeName = "圆柱体";
            Serial.printf("  [形状] 物体在 %d°  %dmm  → %s\n",
                          objAngle, (int)lidar.distanceAt(objAngle), shapeName);
        } else {
            Serial.println("  [形状] 前方未找到物体");
        }
    }
}
