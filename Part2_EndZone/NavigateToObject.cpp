/*
 * NavigateToObject.cpp — 文字驱动导航
 *
 * 流程:
 *   PARALLEL → WAIT_TEXT → CLASSIFY → FIND_TARGET →
 *   SLIDE → APPROACH → VOICE → TURN_HEAD →
 *   WAIT_TEXT → FIND_TARGET → SLIDE → ... (共3轮)
 */

#include "NavigateToObject.h"
#include "lidar_classify.h"
#include <math.h>

#define ROBOT_FRONT    90
#define VOICE_DELAY    2000
#define DROP_MM        300
#define APPROACH_STEPS 4

// 外部依赖
int lidar_findObjects(int* outAngles);
void action_slide(int dir);

// ============================================================
static NTO_Phase g_phase = NTO_WAIT_TEXT;
static int  g_objAngles[3];

// 分类记录
static int  g_posShape[3] = {0, 0, 0};   // 左/中/右 各是什么形状
static int  g_curPos = 1;                 // 当前所在位置

// 文字驱动
static int  g_cycle = 0;                  // 第几轮 0/1/2
static int  g_textShape = 0;              // 收到的文字对应形状
static int  g_targetPos = 0;              // 目标位置
static bool g_textReady = false;          // 收到新文字

// 侧滑
static float g_dropMaxD = 0;
static int  g_dropCount = 0;
static int  g_dropsNeeded = 0;

// 侧滑两段式 (跨2个物体时中间暂停)
static bool         g_slidePause = false;
static unsigned long g_slidePauseStart = 0;

// 侧滑预转弯
static bool g_slidePreTurnDone = false;

// 逼近
static int  g_approachStep = 0;
static bool g_firstApproach = true;

// 语音
static unsigned long g_voiceStart = 0;

// 转头
static unsigned long g_headStart = 0;
static bool g_headTurning = false;

// ============================================================
void nto_reset() {
    g_phase = NTO_WAIT_TEXT;
    g_curPos = 1;
    g_cycle = 0;
    g_textShape = 0;
    g_textReady = false;
    g_dropMaxD = 0;
    g_dropCount = 0;
    g_slidePreTurnDone = false;
    g_firstApproach = true;
    g_headTurning = false;
    for (int i = 0; i < 3; i++) g_posShape[i] = 0;
}

NTO_Phase nto_phase() { return g_phase; }

void nto_textReceived(int shape) {
    if (g_phase == NTO_WAIT_TEXT) {
        g_textShape = shape;
        g_textReady = true;
    }
}

void nto_headTurnDone() {
    if (g_phase == NTO_TURN_HEAD) {
        g_headTurning = false;
        g_textReady = false;
        g_phase = NTO_WAIT_TEXT;
    }
}

void nto_headBackDone() {
    if (g_phase == NTO_HEAD_BACK) {
        g_phase = NTO_FIND_TARGET;
    }
}

void nto_summaryDone() {
    if (g_phase == NTO_SUMMARY) {
        g_phase = NTO_FIND_TARGET;
    }
}

void nto_getObjects(int angles[3], int shapes[3]) {
    for (int i = 0; i < 3; i++) {
        angles[i] = g_objAngles[i];
        shapes[i] = g_posShape[i];
    }
}

int nto_currentShape() {
    return g_posShape[g_curPos];
}

// ============================================================
NavCmd nto_update() {
    NavCmd cmd = {0, 0, false, false};

    // 扫描
    bool needScan = (g_phase != NTO_WAIT_TEXT && g_phase != NTO_VOICE &&
                     g_phase != NTO_TURN_HEAD && g_phase != NTO_HEAD_BACK &&
                     g_phase != NTO_SUMMARY && g_phase != NTO_DONE);
    if (needScan) {
        int objs[3];
        int n = lidar_findObjects(objs);
        if (n >= 3) for (int i = 0; i < 3; i++) g_objAngles[i] = objs[i];
    }

    float d90 = g_map[ROBOT_FRONT];

    switch (g_phase) {

    // ============================================================
    case NTO_GOTO_FIRST:
        g_phase = NTO_WAIT_TEXT;
        return cmd;

    case NTO_PARALLEL:
        g_phase = NTO_WAIT_TEXT;
        return cmd;

    // ============================================================
    case NTO_WAIT_TEXT:
        if (g_textReady) {
            if (g_cycle == 0) {
                // 第1轮头本来就在90°, 直接分类
                g_phase = NTO_CLASSIFY;
            } else {
                // 第2/3轮头偏开了, 先回正
                g_phase = NTO_HEAD_BACK;
                g_headStart = millis();
            }
            Serial.printf("  [文字] 收到 0x%02X\n", g_textShape);
        }
        return cmd;

    // ============================================================
    case NTO_CLASSIFY: {
        static unsigned long classifyStart = 0;
        static int lastRaw[3] = {0,0,0};
        static int sameCnt = 0;

        if (classifyStart == 0) {
            classifyStart = millis();
            lastRaw[0] = lastRaw[1] = lastRaw[2] = 0;
            sameCnt = 0;
            Serial.println("  [分类] 等待5秒(收纸)...");
            return cmd;
        }
        if (millis() - classifyStart < 5000) return cmd;

        // 先检查三个物体的点云质量
        ShapeInfo tmp[3];
        bool badScan = false;
        for (int i = 0; i < 3; i++) {
            classifyObject(g_objAngles[i], &tmp[i]);
            if (tmp[i].distStddev < 0.1f || tmp[i].angularSpan < 5) {
                badScan = true;
                Serial.printf("  [分类] obj%d std=%.1f span=%d 跳过\n",
                              i, tmp[i].distStddev, tmp[i].angularSpan);
            }
        }
        if (badScan) return cmd;  // 质量差, 等下一帧

        int rawShape[3];
        classifyThree(g_objAngles, rawShape);

        // 检查是否与上次一致
        bool same = true;
        for (int i = 0; i < 3; i++) {
            if (rawShape[i] != lastRaw[i]) { same = false; break; }
        }
        if (same) sameCnt++; else sameCnt = 1;
        for (int i = 0; i < 3; i++) lastRaw[i] = rawShape[i];

        Serial.printf("  [分类] (%d/10)\n", sameCnt);
        for (int i = 0; i < 3; i++) {
            const char* names[] = {"左","中","右"};
            Serial.printf("    %s: %s 角度=%d° 距离=%.0fcm std=%.1f 跨度=%d° 边缘=%s\n",
                          names[i],
                          rawShape[i]==1?"球":rawShape[i]==2?"正方体":rawShape[i]==3?"圆柱":"?",
                          g_objAngles[i], g_map[g_objAngles[i]]/10,
                          tmp[i].distStddev, tmp[i].angularSpan,
                          tmp[i].sharpEdges?"陡":"缓");
        }

        if (sameCnt >= 10) {
            for (int i = 0; i < 3; i++) g_posShape[i] = (rawShape[i] > 0) ? (0x10 + rawShape[i]) : 0;
            g_curPos = 1;
            Serial.println("  [分类] 确认! 开始播报总结");
            g_phase = NTO_SUMMARY;
            classifyStart = 0;
        }
        return cmd;
    }

    // ============================================================
    case NTO_SUMMARY:
        // 主程序播报完调 nto_summaryDone()
        return cmd;

    // ============================================================
    case NTO_FIND_TARGET: {
        // 查表: 文字形状在哪个位置
        for (int i = 0; i < 3; i++) {
            if (g_posShape[i] == g_textShape) {
                g_targetPos = i;
                break;
            }
        }
        g_dropsNeeded = abs(g_targetPos - g_curPos);
        g_dropMaxD = 0;
        g_dropCount = 0;
        Serial.printf("  [查表] 文字0x%02X → 位置%d, cur=%d, 需经过%d物体\n",
                      g_textShape, g_targetPos, g_curPos, g_dropsNeeded);

        if (g_dropsNeeded == 0) {
            // 已经在目标位置
            g_phase = NTO_APPROACH;
            g_approachStep = 0;
        } else {
            g_phase = NTO_SLIDE;
            g_slidePreTurnDone = false;
            g_slidePause = false;
        }
        return cmd;
    }

    // ============================================================
    case NTO_SLIDE: {
        if (d90 <= 0) return cmd;

        int dir = (g_targetPos > g_curPos) ? -1 : 1;  // 右滑(-1) 左滑(1)

        // 先做一次预转弯
        if (!g_slidePreTurnDone) {
            g_slidePreTurnDone = true;
            cmd.turnAngle = (dir > 0) ? 10 : -10;
            cmd.smallTurn = true;
            Serial.printf("  [侧滑] 预转弯 %s\n", dir > 0 ? "小左转" : "小右转");
            return cmd;
        }

        // 更新峰值
        if (d90 > g_dropMaxD) g_dropMaxD = d90;

        // 暂停中（跨2物体时到达中间物体，等3秒）
        if (g_slidePause) {
            if (millis() - g_slidePauseStart < 3000) {
                Serial.printf("  [侧滑] 暂停中... %lums\n", millis() - g_slidePauseStart);
                return cmd;
            }
            g_slidePause = false;
            g_dropMaxD = d90;  // 以当前距离为新起点
            Serial.printf("  [侧滑] 暂停结束, 继续后半段\n");
        }

        // 检测跌落
        float drop = g_dropMaxD - d90;
        if (drop > DROP_MM) {
            g_dropCount++;
            g_dropMaxD = d90;  // 重置峰值, 从当前距离开始新的上升
            Serial.printf("  [侧滑] 跌落%d/%d 90°=%.0fcm\n",
                          g_dropCount, g_dropsNeeded, d90/10);

            // 跨2物体: 第一次跌落后暂停 (到中间物体了)
            if (g_dropsNeeded == 2 && g_dropCount == 1) {
                g_slidePause = true;
                g_slidePauseStart = millis();
                Serial.printf("  [侧滑] 到达中间物体, 暂停3秒\n");
                return cmd;
            }
        }

        if (g_dropCount >= g_dropsNeeded) {
            g_curPos = g_targetPos;
            g_phase = NTO_APPROACH;
            g_approachStep = 0;
            g_dropMaxD = 0;
            Serial.printf("  [侧滑] 到达位置%d\n", g_curPos);
            return cmd;
        }

        Serial.printf("  [侧滑] →位置%d %s 90°=%.0fcm 峰值=%.0fcm 跌落%d/%d\n",
                      g_targetPos, dir>0?"左滑":"右滑",
                      d90/10, g_dropMaxD/10, g_dropCount, g_dropsNeeded);
        action_slide(dir);
        return cmd;
    }

    // ============================================================
    case NTO_APPROACH: {
        int need = (g_firstApproach && g_cycle == 0) ? APPROACH_STEPS : 0;

        if (g_approachStep >= need) {
            if (g_cycle == 0) g_firstApproach = false;
            g_phase = NTO_VOICE;
            g_voiceStart = millis();
            Serial.printf("  [逼近] 完成, 播报\n");
            return cmd;
        }
        g_approachStep++;
        Serial.printf("  [逼近] 第%d/%d步 90°=%.0fcm\n", g_approachStep, need, d90/10);
        cmd.forwardMm = 1;
        return cmd;
    }

    // ============================================================
    case NTO_VOICE:
        if (millis() - g_voiceStart < VOICE_DELAY) return cmd;
        g_cycle++;
        if (g_cycle >= 3) {
            g_phase = NTO_DONE;
            cmd.arrived = true;
        } else {
            g_phase = NTO_TURN_HEAD;
            g_headTurning = true;
            g_headStart = millis();
            Serial.println("  [转头] 转180°等下一个文字");
        }
        return cmd;

    // ============================================================
    case NTO_HEAD_BACK:
        // 主程序控制舵机回正, 完成后调 nto_headBackDone()
        return cmd;

    // ============================================================
    case NTO_TURN_HEAD:
        // 主程序控制舵机偏头(0°), 完成后调 nto_headTurnDone()
        return cmd;

    // ============================================================
    case NTO_DONE:
        cmd.arrived = true;
        return cmd;
    }

    return cmd;
}
