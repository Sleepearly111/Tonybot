#include <Arduino.h>
#include "lidar/LidarDriver.h"
#include "LobotServoController.h"
#include "wdt_util.h"

// ============================================================
// 模式切换 — 注释掉下面这行切回形状测试模式
// ============================================================
#define LIDAR_STREAM_MODE    // 串口流式输出 → PC端 lidar_viewer.py 可视化

// ============================================================
// 引脚
// ============================================================
#define PIN_LIDAR_RX     32
#define PIN_LIDAR_TX     33
#define PIN_LIDAR_MOTOR  13

LidarDriver lidar;
extern LobotServoController Controller;

// A1M8 安装偏移：雷达 90° = 机器人 0°（前方）
#define HEADING_OFFSET  90

static int toRobot(int lidarDeg) { return (lidarDeg - HEADING_OFFSET + 360) % 360; }
static int toLidar(int robotDeg) { return (robotDeg + HEADING_OFFSET) % 360; }
static float robotDistAt(int rDeg) { return lidar.distanceAt(toLidar(rDeg)); }

static const char* shapeName(uint8_t id) {
    switch (id) {
        case 1:  return "球体";
        case 2:  return "正方体";
        case 3:  return "圆柱体";
        default: return "未知";
    }
}

// ============================================================
void setup() {
    Serial.begin(115200);
    safeDelay(500);

#ifdef LIDAR_STREAM_MODE
    Serial.println("\n========== LiDAR STREAM MODE ==========");
    Serial.println("  运行: python tools/lidar_viewer.py <COM口>");
    Serial.println("========================================");
#else
    Serial.println("\n========================================");
    Serial.println("  形状识别测试");
    Serial.println("========================================\n");
#endif

    // 舵机初始化 → 直立
    Serial.println("[INIT] 舵机直立...");
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    safeDelay(100);
    Controller.runActionGroup(0, 1);  // stand
    safeDelay(2000);

    // LiDAR
    Serial.println("[INIT] 激光雷达...");
    if (!lidar.begin(PIN_LIDAR_RX, PIN_LIDAR_TX, PIN_LIDAR_MOTOR)) {
        Serial.println("[FATAL] 雷达初始化失败");
        while (1) safeDelay(1000);
    }
    Serial.println("[OK] 雷达就绪\n");
}

// ============================================================
#ifdef LIDAR_STREAM_MODE

// === 流式输出模式 ===
void loop() {
    lidar.update();

    if (lidar.isScanComplete()) {
        lidar.resetScanFlag();
        lidar.printScanStream();
    }
}

#else

// === 形状识别测试模式 ===

// 在前方扇形区域找物体，返回找到的数量和角度
static int findObjects(int* outAngles, int maxObj, int sectorW = 60) {
    int found = 0;
    for (int i = 0; i <= 2 * sectorW; i++) {
        int degL = toLidar(360 - sectorW + i);
        float d = lidar.distanceAt(degL);
        if (d < 100 || d > 2000) continue;

        if (found == 0) {
            outAngles[found++] = toRobot(degL);
        } else {
            int prevL = toLidar(outAngles[found - 1]);
            float prevD = lidar.distanceAt(prevL);
            int gap = (degL - prevL + 360) % 360;
            if (gap > 8 || fabs(d - prevD) > 300) {
                if (found < maxObj) {
                    outAngles[found++] = toRobot(degL);
                }
            }
        }
    }
    return found;
}

static unsigned long lastPrintMs = 0;

void loop() {
    lidar.update();

    if (!lidar.isScanComplete()) return;
    lidar.resetScanFlag();

    if (millis() - lastPrintMs < 3000) return;
    lastPrintMs = millis();

    // 扫描前方 ±60°
    int angles[8];
    int n = findObjects(angles, 8, 60);

    Serial.print("-------- ");
    Serial.print(millis() / 1000);
    Serial.print("s  (检测到 ");
    Serial.print(n);
    Serial.println(" 个物体) --------");

    for (int i = 0; i < n; i++) {
        float dist = robotDistAt(angles[i]);
        LidarDriver::ShapeInfo info;
        int shape = lidar.classifyObjectAt(toLidar(angles[i]), &info);

        const char* side = (angles[i] < 0) ? "左" : "右";
        Serial.printf("  [%s侧] 角度=%d°  距离=%.0fmm  形状=%s\n",
                      side, abs(angles[i]), dist, shapeName(shape));
        Serial.printf("         span=%d°  std=%.1fmm  边缘陡峭=%s\n",
                      info.angularSpan, info.distStddev,
                      info.sharpEdges ? "Y" : "N");
    }

    if (n == 0) {
        Serial.println("  (前方无物体)");
        Serial.printf("  [原始] 0°=%.0f  -30°=%.0f  30°=%.0f  -60°=%.0f  60°=%.0f\n",
                      robotDistAt(0), robotDistAt(-30), robotDistAt(30),
                      robotDistAt(-60), robotDistAt(60));
    }
    Serial.println();
}

#endif
