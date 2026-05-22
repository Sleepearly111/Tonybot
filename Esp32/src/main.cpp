#include <Arduino.h>
#include "lidar/LidarDriver.h"
#include "LobotServoController.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// ============================================================
// A1M8 激光雷达引脚
// ============================================================
#define PIN_LIDAR_RX     32
#define PIN_LIDAR_TX     33
#define PIN_LIDAR_MOTOR  13

LidarDriver lidar;
extern LobotServoController Controller;

// ============================================================
// 行走测试阶段
// ============================================================
enum WalkPhase { WALK_INIT, WALK_GOING, WALK_DONE };
static WalkPhase phase = WALK_INIT;
static unsigned long phaseTimer = 0;

// 角度偏移：雷达安装偏右 90°
// 机器人 0°(前方) = 雷达 90°(右方)
#define HEADING_OFFSET  90

// 雷达坐标系 ↔ 机器人坐标系
static int toRobot(int lidarDeg) { return (lidarDeg - HEADING_OFFSET + 360) % 360; }
static int toLidar(int robotDeg) { return (robotDeg + HEADING_OFFSET) % 360; }

// 机器人坐标系下的 Lidar 查询封装
static int robotFindNearest(int rStart, int rEnd) {
    int result = lidar.findNearest(toLidar(rStart), toLidar(rEnd));
    return (result >= 0) ? toRobot(result) : -1;
}
static float robotDistanceAt(int rDeg) { return lidar.distanceAt(toLidar(rDeg)); }

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

    // 喂狗（保留硬件看门狗，避免系统真正挂死时无法恢复）
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG1.wdt_feed = 1;
    TIMERG1.wdt_wprotect = 0;
    delay(500);
    Serial.println("\n========================================");
    Serial.println("  行走 + 形状识别测试");
    Serial.println("========================================\n");

    Serial.println("[PHASE] 初始化舵机控制器...");
    Serial2.begin(115200, SERIAL_8N1, 16, 17);  // 舵机总线  RX=16 TX=17
    delay(100);
    Controller.runActionGroup(0, 1);  // 先直立
    delay(2000);  // 等直立完成

    // 进雷达初始化前再喂一次狗，避免内部长 delay 超时
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG1.wdt_feed = 1;
    TIMERG1.wdt_wprotect = 0;

    if (!lidar.begin(PIN_LIDAR_RX, PIN_LIDAR_TX, PIN_LIDAR_MOTOR)) {
        Serial.println("[FATAL] 雷达初始化失败！检查接线和供电。");
        while (1) delay(1000);
    }
    Serial.println("[OK] 激光雷达已就绪\n");

    // 开始行走测试
    Serial.println("[PHASE] 前进 2 步...");
    Controller.runActionGroup(25, 2);  // 25=forward, 走2步
    phase = WALK_GOING;
    phaseTimer = millis();
}

static unsigned long lastPrintMs = 0;

// ============================================================
void loop() {
    lidar.update();

    // — 行走阶段：等 3 秒让动作组执行完 —
    if (phase == WALK_GOING) {
        if (millis() - phaseTimer >= 3000) {
            Serial.println("[PHASE] 停止，直立");
            Controller.runActionGroup(0, 1);  // 立正
            phase = WALK_DONE;
            phaseTimer = millis();
            Serial.println("[PHASE] 行走完成，开始形状识别\n");
        }
        return;  // 行走阶段不做雷达识别
    }

    // 行走完成后停 1 秒再开始识别
    if (phase == WALK_DONE) {
        if (millis() - phaseTimer < 1000) return;
        phase = WALK_DONE;  // 保持，跳过这个 if
    }

    if (!lidar.isScanComplete()) return;
    lidar.resetScanFlag();

    // 每 2 秒报告一次
    if (millis() - lastPrintMs < 2000) return;
    lastPrintMs = millis();

    // ---- 前方 ±45° 范围内找物体 ----
    const int SECTOR_W = 45;
    int objDeg = robotFindNearest(360 - SECTOR_W, SECTOR_W);

    Serial.print("-------- ");
    Serial.print(millis() / 1000);
    Serial.println("s --------");

    if (objDeg < 0) {
        Serial.println("[前方] 未检测到物体");
        float dF = robotDistanceAt(0);
        float dL = robotDistanceAt(30);
        float dR = robotDistanceAt(330);
        Serial.printf("[前方] 0°=%.0fmm  左30°=%.0fmm  右30°=%.0fmm\n", dF, dL, dR);
        Serial.println();
        return;
    }

    float dist = robotDistanceAt(objDeg);
    Serial.printf("[物体] 角度=%d°  距离=%.0fmm\n", objDeg, dist);

    if (dist > 1500) {
        Serial.println("[跳过] 距离 > 1.5m，太远不分类\n");
        return;
    }

    // ---- 形状分类 ----
    LidarDriver::ShapeInfo info;
    int shape = lidar.classifyObjectAt(toLidar(objDeg), &info);

    Serial.printf("[形状] %s  span=%d°  std=%.1fmm  边缘陡峭=%s\n",
                  shapeName(shape), info.angularSpan, info.distStddev,
                  info.sharpEdges ? "Y" : "N");
    Serial.printf("[细节] 中心角=%d°  最近=%.0fmm  最远=%.0fmm\n",
                  info.centerAngle, info.minDist, info.maxDist);
    Serial.println();
}
