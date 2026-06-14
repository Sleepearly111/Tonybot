/*
 * main.cpp — CRAIC 2026 比赛状态机
 *
 * Task 1 (5min):       通道一 → 避障区S型绕柱 → 找通道二入口
 * Task 2+3 (共5min):   通道二 → 终点区(一次性读标签+LiDAR分类3物体)
 *                       → 依次走到物体1/2/3标记区播报
 *
 * 赛前修改:
 *   PILLAR_ROUTE    — ROUTE_RED_LEFT / ROUTE_RED_RIGHT / ROUTE_BLUE_LEFT / ROUTE_BLUE_RIGHT
 *   CIRCLE_ROUNDS   — 每柱绕行圈数
 */

#include <Arduino.h>
#include "lidar/LidarDriver.h"
#include "LobotServoController.h"
#include "hw_esp32cam_ctl.h"
#include "voice/HWSensor.h"
#include "voice/ShapeVoicePlayer.h"
#include "Hiwonder.hpp"
#include "Servo.h"
#include "wdt_util.h"
#include "tracking/HeadTracker.hpp"
#include "tracking/ObjectFollower.hpp"
#include "tracking/CirclePillar.hpp"
#include "tracking/PillarRoute.hpp"
#include "fall/FallDetector.hpp"

// ============================================================
// 比赛配置
// ============================================================
#define PILLAR_ROUTE       ROUTE_RED_LEFT
#define CIRCLE_ROUNDS      4
#define TASK1_TIMEOUT_MS   300000
#define TASK23_TIMEOUT_MS  300000

#define HEADING_OFFSET     90
#define CHANNEL_STEPS       8

#define ENDZONE_TARGET      800
#define ENDZONE_TOLERANCE   200
#define MARKER_TARGET       200

#define ACT_STAND           0
#define ACT_FORWARD         25
#define ACT_TURN_LEFT       23
#define ACT_TURN_RIGHT      24

// ============================================================
// 全局对象 (定义在 globals.cpp)
// ============================================================
extern LobotServoController Controller;
extern HWSensor hwsensor;
extern Servo sonarServo;
extern IMU imu;

LidarDriver      lidar;
HW_ESP32Cam      cam;
ShapeVoicePlayer voicePlayer;

// ============================================================
// 角度工具
// ============================================================
static inline int toLidar(int robotDeg)  { return (robotDeg + HEADING_OFFSET) % 360; }
static inline int toRobot(int lidarDeg)  { return (lidarDeg - HEADING_OFFSET + 360) % 360; }
static inline int wrap360(int d)         { return (d + 360) % 360; }

// ============================================================
// 比赛阶段
// ============================================================
enum class Phase {
    INIT,
    // Task 1 — 避障绕柱 (5min)
    T1_CHANNEL1,
    T1_APPROACH,
    T1_CIRCLE,
    T1_CLEAR,
    T1_FIND_CH2,
    // Task 2+3 — 识别+播报 (共5min)
    T2_CHANNEL2,         // 通道二穿越
    T2_TO_ENDZONE,       // 前进到终点区
    T2_READ_AND_CLASSIFY,// 读标签 + LiDAR分类全部3物体 (停在终点区)
    T3_NAV_MARKER,       // 走向当前物体的标记区
    T3_ANNOUNCE,         // 播报当前物体, 然后去下一个或结束
    DONE,
    ERROR
};

Phase        g_phase        = Phase::INIT;
bool         g_phaseEntered = true;
unsigned long g_phaseStart  = 0;
unsigned long g_taskStart   = 0;

// 3个物体的识别结果 (在 T2_READ_AND_CLASSIFY 一次性填好)
int          g_targetShapes[3] = {0, 0, 0};  // 1=球体 2=正方体 3=圆柱体
int          g_targetAngles[3] = {-1,-1,-1}; // robot-relative 角度
int          g_visitIndex      = 0;          // 当前正在处理第几个物体 (0/1/2)

// 通用计数器
static int   g_stepCount    = 0;
static int   g_stableCount  = 0;

// ============================================================
// 前向声明
// ============================================================
static void enterPhase(Phase p);
static const char* phaseName(Phase p);
static const char* labelName(int label);
static const char* shapeName(int shape);
static int  findThreeObjects(int outAngles[3]);
static int  constellationOffset();
static bool isAtEndZone();
static int  classifyWithVoting(int robotAngle, int requiredVotes = 3);

// ============================================================
// 状态切换
// ============================================================
static void enterPhase(Phase p) {
    g_phase = p;
    g_phaseEntered = true;
    g_phaseStart = millis();
    g_stepCount = 0;
    g_stableCount = 0;
    Serial.printf("\n[状态机] → %s  [%lus]\n", phaseName(p), millis() / 1000);
}

static const char* phaseName(Phase p) {
    switch (p) {
        case Phase::INIT:               return "初始化";
        case Phase::T1_CHANNEL1:        return "T1-通道一穿越";
        case Phase::T1_APPROACH:        return "T1-接近柱子";
        case Phase::T1_CIRCLE:          return "T1-绕柱";
        case Phase::T1_CLEAR:           return "T1-脱离柱子";
        case Phase::T1_FIND_CH2:        return "T1-找通道二";
        case Phase::T2_CHANNEL2:        return "T2-通道二穿越";
        case Phase::T2_TO_ENDZONE:      return "T2-到达终点区";
        case Phase::T2_READ_AND_CLASSIFY: return "T2-读标签+分类";
        case Phase::T3_NAV_MARKER:      return "T3-走向标记区";
        case Phase::T3_ANNOUNCE:        return "T3-播报";
        case Phase::DONE:               return "完成";
        case Phase::ERROR:              return "错误";
        default: return "?";
    }
}

static const char* labelName(int label) {
    switch (label) {
        case 0x11: return "球体";
        case 0x12: return "正方体";
        case 0x13: return "圆柱体";
        default:    return "未知";
    }
}

static const char* shapeName(int shape) {
    switch (shape) {
        case 1: return "球体";
        case 2: return "正方体";
        case 3: return "圆柱体";
        default:return "未知";
    }
}

// ============================================================
// LiDAR 辅助
// ============================================================

static int findThreeObjects(int outAngles[3]) {
    struct { int start, end, count; } clusters[16];
    int nClust = 0;
    bool inside = false;

    for (int r = -90; r <= 90; r++) {
        float d = lidar.distanceAt(toLidar(wrap360(r)));
        if (d > 100 && d < 2000) {
            if (!inside) {
                inside = true;
                if (nClust < 16) { clusters[nClust].start = r; clusters[nClust].count = 0; }
            }
            if (nClust < 16) { clusters[nClust].end = r; clusters[nClust].count++; }
        } else {
            if (inside) { inside = false; if (nClust < 16) nClust++; }
        }
    }
    if (inside && nClust < 16) nClust++;

    int idx[16];
    for (int i = 0; i < nClust; i++) idx[i] = i;
    for (int i = 0; i < nClust; i++)
        for (int j = i + 1; j < nClust; j++)
            if (clusters[idx[j]].count > clusters[idx[i]].count)
                { int t = idx[i]; idx[i] = idx[j]; idx[j] = t; }

    int found = 0;
    for (int i = 0; i < nClust && found < 3; i++) {
        auto& c = clusters[idx[i]];
        if (c.count < 4) continue;
        int ang = (c.start + c.end) / 2;
        int j = found;
        while (j > 0 && outAngles[j - 1] > ang) { outAngles[j] = outAngles[j - 1]; j--; }
        outAngles[j] = ang;
        found++;
    }
    return found;
}

static int constellationOffset() {
    int objs[3];
    if (findThreeObjects(objs) < 3) return 999;
    return objs[1];
}

static bool isAtEndZone() {
    int objs[3];
    int n = findThreeObjects(objs);
    if (n < 1) return false;
    int mid = (n == 1) ? objs[0] : (n == 2)
        ? (abs(objs[0]) < abs(objs[1]) ? objs[0] : objs[1]) : objs[1];
    float d = lidar.distanceAt(toLidar(wrap360(mid)));
    return (d >= ENDZONE_TARGET - ENDZONE_TOLERANCE &&
            d <= ENDZONE_TARGET + ENDZONE_TOLERANCE);
}

static int classifyWithVoting(int robotAngle, int requiredVotes) {
    int votes[4] = {0};
    for (int i = 0; i < 10; i++) {
        unsigned long t0 = millis();
        while (!lidar.isScanComplete()) {
            lidar.update();
            if (millis() - t0 > 3000) break;
        }
        lidar.resetScanFlag();
        int shape = lidar.classifyObjectAt(toLidar(wrap360(robotAngle)));
        if (shape >= 1 && shape <= 3) {
            votes[shape]++;
            if (votes[shape] >= requiredVotes) return shape;
        }
    }
    int best = 1;
    for (int i = 2; i <= 3; i++) if (votes[i] > votes[best]) best = i;
    return (votes[best] > 0) ? best : 0;
}

// ============================================================
// setup
// ============================================================
void setup() {
    Serial.begin(115200);
    safeDelay(500);
    Serial.println("\n========================================");
    Serial.println("  CRAIC 2026 — Tonybot 比赛状态机");
    Serial.println("========================================");

    Serial.println("[INIT] I2C + ESP32-S3...");
    Wire.begin(22, 23);
    cam.begin();
    safeDelay(100);

    Serial.println("[INIT] 舵机总线 Serial2...");
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    safeDelay(100);
    Controller.runActionGroup(ACT_STAND, 1);
    safeDelay(2000);

    Serial.println("[INIT] 激光雷达...");
    if (!lidar.begin(32, 33, 13)) {
        Serial.println("[FATAL] 雷达初始化失败");
        enterPhase(Phase::ERROR);
        return;
    }

    Serial.println("[INIT] IMU...");
    imu.begin();
    safeDelay(500);

    Serial.println("[INIT] 头部追踪 + 绕柱路线...");
    tracker_init();
    pillarRoute_start(PILLAR_ROUTE);
    pillarRoute_setRounds(CIRCLE_ROUNDS, CIRCLE_ROUNDS);
    safeDelay(500);

    fallDetector_init();

    // Task1: S3 设为颜色追踪模式
    cam.setMode(0x01);
    safeDelay(100);

    Serial.println("\n[就绪] 比赛开始!\n");
    g_visitIndex = 0;
    enterPhase(Phase::T1_CHANNEL1);
    g_taskStart = millis();
}

// ============================================================
// loop
// ============================================================
void loop() {
    // WDT feed
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG1.wdt_feed = 1;
    TIMERG1.wdt_wprotect = 0;

    // 跌倒检测
    fallDetector_update();
    if (fallDetector_isRecovering()) { safeDelay(50); return; }

    // LiDAR
    lidar.update();

    // ============ 超时保护 ============
    if (g_phase != Phase::DONE && g_phase != Phase::ERROR && g_phase != Phase::INIT) {
        bool isTask1 = (g_phase <= Phase::T1_FIND_CH2);
        unsigned long timeout = isTask1 ? TASK1_TIMEOUT_MS : TASK23_TIMEOUT_MS;

        if (millis() - g_taskStart > timeout) {
            Serial.printf("[超时] %s 时间到\n", isTask1 ? "Task1" : "Task2+3");
            if (isTask1) {
                g_taskStart = millis();
                enterPhase(Phase::T2_CHANNEL2);
            } else {
                enterPhase(Phase::ERROR);
            }
            return;
        }
    }

    // ============ 状态机 ============
    switch (g_phase) {

    // ================================================================
    // INIT
    // ================================================================
    case Phase::INIT:
        break;

    // ================================================================
    // T1: 通道一穿越
    // ================================================================
    case Phase::T1_CHANNEL1: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.println("[T1-通道一] 穿越500mm通道");
        }
        if (!lidar.isScanComplete()) break;
        lidar.resetScanFlag();

        int offset = constellationOffset();
        if (offset != 999 && abs(offset) > 15) {
            Controller.runActionGroup((offset > 0) ? ACT_TURN_LEFT : ACT_TURN_RIGHT, 1);
            safeDelay(1200);
        }

        Controller.runActionGroup(ACT_FORWARD, 1);
        safeDelay(1000);
        g_stepCount++;

        if (g_stepCount >= CHANNEL_STEPS) {
            Serial.println("[T1-通道一] 完成");
            enterPhase(Phase::T1_APPROACH);
        }
        break;
    }

    // ================================================================
    // T1: 接近柱子
    // ================================================================
    case Phase::T1_APPROACH: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.printf("[T1-接近] 颜色=%d\n", pillarRoute_color());
            objectFollower_init();
        }

        objectFollower_update(pillarRoute_color(), 20);

        static int lastW = 0, wStable = 0;
        int w = get_width();
        if (w > 200) { wStable = (w == lastW) ? wStable + 1 : 1; }
        else         { wStable = 0; }
        lastW = w;

        if (wStable >= 3) {
            Serial.println("[T1-接近] 已靠近柱子");
            enterPhase(Phase::T1_CIRCLE);
        }
        break;
    }

    // ================================================================
    // T1: 绕柱
    // ================================================================
    case Phase::T1_CIRCLE: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.printf("[T1-绕柱] 颜色=%d 方向=%d 圈数=%d\n",
                          pillarRoute_color(), pillarRoute_direction(), pillarRoute_rounds());
            circlePillar_init(pillarRoute_rounds());
        }

        circlePillar_update(pillarRoute_color(), pillarRoute_direction());

        if (circlePillar_isDone()) {
            Serial.println("[T1-绕柱] 完成");
            enterPhase(Phase::T1_CLEAR);
        }
        break;
    }

    // ================================================================
    // T1: 脱离柱子, 切下一根或进Task2
    // ================================================================
    case Phase::T1_CLEAR: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.println("[T1-脱离] 前走3步");
        }

        if (g_stepCount < 3) {
            Controller.runActionGroup(ACT_FORWARD, 1);
            safeDelay(1000);
            g_stepCount++;
            break;
        }

        if (pillarRoute_next()) {
            Serial.println("[T1] 两柱绕完!");
            enterPhase(Phase::T1_FIND_CH2);
        } else {
            Serial.printf("[T1] 下一根: 颜色=%d 方向=%d\n",
                          pillarRoute_color(), pillarRoute_direction());
            enterPhase(Phase::T1_APPROACH);
        }
        break;
    }

    // ================================================================
    // T1: 找通道二入口
    // ================================================================
    case Phase::T1_FIND_CH2: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.println("[T1-找CH2] LiDAR扫描3物体方向...");
        }
        if (!lidar.isScanComplete()) break;
        lidar.resetScanFlag();

        int offset = constellationOffset();
        if (offset == 999) {
            Controller.runActionGroup(ACT_TURN_LEFT, 1);
            safeDelay(1200);
            g_stepCount++;
            if (g_stepCount > 12) {
                Serial.println("[T1-找CH2] 搜索超时, 进入Task2+3");
                g_taskStart = millis();
                g_visitIndex = 0;
                enterPhase(Phase::T2_CHANNEL2);
            }
            break;
        }

        if (abs(offset) <= 10) {
            Serial.println("[T1-找CH2] 已对准通道二");
            g_taskStart = millis();
            g_visitIndex = 0;
            enterPhase(Phase::T2_CHANNEL2);
        } else {
            Controller.runActionGroup((offset > 0) ? ACT_TURN_LEFT : ACT_TURN_RIGHT, 1);
            safeDelay(1200);
        }
        break;
    }

    // ================================================================
    // T2: 通道二穿越
    // ================================================================
    case Phase::T2_CHANNEL2: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            // Task2+3: S3 切到 AI 标签识别模式
            cam.setMode(0x02);
            Serial.println("[T2-通道二] 穿越500mm (S3→AI模式)");
        }
        if (!lidar.isScanComplete()) break;
        lidar.resetScanFlag();

        int offset = constellationOffset();
        if (offset != 999 && abs(offset) > 15) {
            Controller.runActionGroup((offset > 0) ? ACT_TURN_LEFT : ACT_TURN_RIGHT, 1);
            safeDelay(1200);
        }

        Controller.runActionGroup(ACT_FORWARD, 1);
        safeDelay(1000);
        g_stepCount++;

        if (g_stepCount >= CHANNEL_STEPS) {
            Serial.println("[T2-通道二] 完成");
            enterPhase(Phase::T2_TO_ENDZONE);
        }
        break;
    }

    // ================================================================
    // T2: 前进到终点区 (~800mm)
    // ================================================================
    case Phase::T2_TO_ENDZONE: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.println("[T2-终点区] 前进到距离~800mm");
        }
        if (!lidar.isScanComplete()) break;
        lidar.resetScanFlag();

        if (isAtEndZone()) {
            g_stableCount++;
            if (g_stableCount >= 3) {
                Serial.println("[T2-终点区] 已到达!");
                Controller.runActionGroup(ACT_STAND, 1);
                safeDelay(1500);
                enterPhase(Phase::T2_READ_AND_CLASSIFY);
            }
        } else {
            g_stableCount = 0;
            int offset = constellationOffset();
            if (offset != 999 && abs(offset) > 15) {
                Controller.runActionGroup((offset > 0) ? ACT_TURN_LEFT : ACT_TURN_RIGHT, 1);
                safeDelay(1200);
            }
            Controller.runActionGroup(ACT_FORWARD, 1);
            safeDelay(1000);
        }
        break;
    }

    // ================================================================
    // T2: 在终点区读标签 + LiDAR分类3物体 (一次性完成)
    //     标签裁判可能逐个展示, 连续读取直到收集3个不同结果
    //     LiDAR对3个物体分别分类投票
    // ================================================================
    case Phase::T2_READ_AND_CLASSIFY: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.println("[T2-读+分类] 读取标签 + LiDAR分类3物体...");
            // 重置数组
            for (int i = 0; i < 3; i++) { g_targetShapes[i] = 0; g_targetAngles[i] = -1; }
        }

        // --- Step 0: LiDAR找到3个物体角度 ---
        int objs[3];
        if (findThreeObjects(objs) < 3) {
            if (!lidar.isScanComplete()) break;
            lidar.resetScanFlag();
            // 没找到3物体, 继续等
            break;
        }

        // --- Step 1: 连续读标签, 收集3个不重复的结果 ---
        // 标签值: 0x11=球体 0x12=正方体 0x13=圆柱体
        static int collectedLabels[3] = {0, 0, 0};
        static int labelCount = 0;

        if (labelCount < 3) {
            uint8_t label = cam.readAILabel();
            if (label == 0x11 || label == 0x12 || label == 0x13) {
                // 检查是否已收集
                bool dup = false;
                for (int i = 0; i < labelCount; i++) {
                    if (collectedLabels[i] == label) { dup = true; break; }
                }
                if (!dup) {
                    collectedLabels[labelCount++] = label;
                    Serial.printf("[T2-标签] 收集到 %d/3: %s\n", labelCount, labelName(label));
                }
            }
            // 如果还没收集够3个, 等待下一帧
            if (labelCount < 3) break;
        }

        // --- Step 2: 对3个物体分别分类 ---
        Serial.println("[T2-分类] LiDAR分类3物体...");
        for (int i = 0; i < 3; i++) {
            g_targetAngles[i] = objs[i];
            // 投票确认每个物体的形状
            g_targetShapes[i] = classifyWithVoting(objs[i], 3);
            Serial.printf("  [%d] angle=%d° → %s\n",
                          i, objs[i], shapeName(g_targetShapes[i]));
        }

        // --- Step 3: 打印映射 ---
        Serial.println("[T2-读+分类] 结果映射:");
        for (int i = 0; i < 3; i++) {
            int labelShape = 0;
            if (collectedLabels[i] == 0x11) labelShape = 1;
            else if (collectedLabels[i] == 0x12) labelShape = 2;
            else if (collectedLabels[i] == 0x13) labelShape = 3;

            // 在3个物体中找匹配的
            for (int j = 0; j < 3; j++) {
                if (g_targetShapes[j] == labelShape) {
                    Serial.printf("  标签=%s → 物体%d (angle=%d°)\n",
                                  labelName(collectedLabels[i]), j + 1, g_targetAngles[j]);
                    break;
                }
            }
        }

        // --- Step 4: 按左→中→右顺序排列访问顺序 ---
        // Angles are already sorted (objs[0]=leftmost, objs[1]=center, objs[2]=rightmost)
        // We'll visit in order: 0, 1, 2
        Serial.println("[T2-读+分类] 准备依次访问3个标记区");
        g_visitIndex = 0;
        Serial.printf("[T2-读+分类] 第1个目标: angle=%d° shape=%s\n",
                      g_targetAngles[0], shapeName(g_targetShapes[0]));
        enterPhase(Phase::T3_NAV_MARKER);
        break;
    }

    // ================================================================
    // T3: 走向当前物体的标记区 (~200mm)
    // ================================================================
    case Phase::T3_NAV_MARKER: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            int idx = g_visitIndex;
            Serial.printf("[T3-导航] 物体%d/3: angle=%d° → %dmm\n",
                          idx + 1, g_targetAngles[idx], MARKER_TARGET);
        }

        int idx = g_visitIndex;
        if (!lidar.isScanComplete()) break;
        lidar.resetScanFlag();

        float dist = lidar.distanceAt(toLidar(wrap360(g_targetAngles[idx])));
        if (dist <= 0) {
            Serial.println("[T3-导航] 目标丢失, 扫描中...");
            break;
        }

        Serial.printf("[T3-导航] 距离=%.0fmm\n", dist);

        if (dist <= MARKER_TARGET + 50) {
            Controller.runActionGroup(ACT_STAND, 1);
            safeDelay(1500);
            Serial.printf("[T3-导航] 到达物体%d标记区!\n", idx + 1);
            enterPhase(Phase::T3_ANNOUNCE);
            break;
        }

        // 对准
        if (abs(g_targetAngles[idx]) > 10) {
            Controller.runActionGroup((g_targetAngles[idx] > 0) ? ACT_TURN_LEFT : ACT_TURN_RIGHT, 1);
            safeDelay(1200);
        }
        // 前进
        int steps = (dist > 800) ? 3 : 1;
        Controller.runActionGroup(ACT_FORWARD, steps);
        safeDelay(steps * 800);
        break;
    }

    // ================================================================
    // T3: 播报当前物体, 然后去下一个或结束
    // ================================================================
    case Phase::T3_ANNOUNCE: {
        if (g_phaseEntered) {
            g_phaseEntered = false;
            int idx = g_visitIndex;
            Serial.printf("[T3-播报] 物体%d/3: %s\n",
                          idx + 1, shapeName(g_targetShapes[idx]));
        }

        // 播报 (ShapeVoicePlayer 内部播3次)
        int idx = g_visitIndex;
        switch (g_targetShapes[idx]) {
            case 1:  voicePlayer.playSphere();    break;
            case 2:  voicePlayer.playCube();      break;
            case 3:  voicePlayer.playCylinder();  break;
            default: break;
        }
        safeDelay(4000);

        Serial.printf("[T3-播报] 物体%d播报完成\n", idx + 1);
        g_visitIndex++;

        if (g_visitIndex >= 3) {
            Serial.println("[T3] 3个物体全部播报完成! 比赛结束!");
            enterPhase(Phase::DONE);
        } else {
            // 直接去下一个物体, 不返回终点区
            Serial.printf("[T3] 直接走向物体%d/3...\n", g_visitIndex + 1);
            enterPhase(Phase::T3_NAV_MARKER);
        }
        break;
    }

    // ================================================================
    // DONE / ERROR
    // ================================================================
    case Phase::DONE:
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.println("\n========================================");
            Serial.println("  比赛完成! 3个物体全部识别播报完毕");
            Serial.printf("  总用时: %lus\n", millis() / 1000);
            Serial.println("========================================");
        }
        safeDelay(1000);
        break;

    case Phase::ERROR:
        if (g_phaseEntered) {
            g_phaseEntered = false;
            Serial.println("[ERROR] 停止");
        }
        safeDelay(1000);
        break;
    }
}
