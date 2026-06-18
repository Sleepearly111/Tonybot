/*
 * Part2_EndZone.ino — CRAIC 2026 终点区导航 + 文字驱动分类
 *
 * 前置条件: Part1 绕柱已完成，机器人立正站立
 *
 * 流程:
 *   STAGE_NAV      → LiDAR 导航到终点区 (80cm)
 *   STAGE_ARRIVED  → 等待裁判文字指令 (S3 AI模式)
 *   STAGE_CLASSIFY → 终点区 3 轮文字驱动分类
 *   STAGE_DONE     → 全部完成
 *
 * 硬件:
 *   Serial1 (32/33) = LiDAR
 *   Serial2 (17/16) = 舵机控制板
 *   I2C   (22/23)   = S3摄像头(0x52) + 语音(0x34) + 超声波(0x77) + IMU
 *   IO4             = 头部舵机
 *   IO13            = LiDAR 电机
 *   IO21            = 蜂鸣器
 */

// ============================================================
// 基础库
// ============================================================
#include "base_config.h"
#include "HardwareSerial.h"
#include "LobotServoController.h"
#include "Arduino.h"
#include "Servo.h"
#include "HWSensor.h"
#include "Hiwonder.hpp"

// ============================================================
// LiDAR + 分类阶段模块
// ============================================================
#include "lidar_common.h"
#include "lidar_navigate.h"
#include "lidar_endzone.h"
#include "lidar_action.h"
#include "NavigateToObject.h"
#include "lidar_classify.h"

// ============================================================
// 全局对象
// ============================================================
LobotServoController Controller(Serial2);
HWSensor hwsensor;
Servo sonarServo;

// ============================================================
// S3 通信常量
// ============================================================
#define ESP32S3_I2C_ADDR  0x52
#define ESP32S3_REG_LABEL 0x10
#define ESP32S3_REG_MODE  0x30
#define ESP32S3_MODE_AI   0x02

// ============================================================
// 阶段
// ============================================================
enum MainStage {
    STAGE_NAV,         // LiDAR 导航到终点区
    STAGE_ARRIVED,     // 等裁判文字
    STAGE_CLASSIFY,    // 终点区 3 轮分类
    STAGE_DONE         // 完成
};
static MainStage g_stage = STAGE_NAV;

// ============================================================
// S3 辅助函数
// ============================================================
static void s3SetMode(uint8_t mode) {
    Wire.beginTransmission(ESP32S3_I2C_ADDR);
    Wire.write(ESP32S3_REG_MODE);
    Wire.write(mode);
    Wire.endTransmission();
}

static int readTextFromESP32() {
    Wire.beginTransmission(ESP32S3_I2C_ADDR);
    Wire.write(ESP32S3_REG_LABEL);
    if (Wire.endTransmission(false) != 0) return 0;
    Wire.requestFrom(ESP32S3_I2C_ADDR, 1);
    if (Wire.available()) return Wire.read();
    return 0;
}

static const char* shapeName(int s) {
    switch (s) {
        case 0x11: return "球体";
        case 0x12: return "正方体";
        case 0x13: return "圆柱体";
        default:   return "?";
    }
}

static const char* stageName(MainStage s) {
    switch (s) {
        case STAGE_NAV:      return "导航";
        case STAGE_ARRIVED:  return "等文字";
        case STAGE_CLASSIFY: return "分类";
        case STAGE_DONE:     return "完成";
        default: return "?";
    }
}

// ============================================================
// setup
// ============================================================
void setup() {
    Serial.begin(115200);
    action_wait(500);
    Serial.println("\n========================================");
    Serial.println("  Part2: 终点区导航 + 文字分类");
    Serial.println("========================================\n");

    // ---- Serial2: 舵机控制板 ----
    Serial2.begin(9600, SERIAL_8N1, IO_BaseRX, IO_BaseTX);
    action_wait(100);

    // ---- Lidar 动作执行器 ----
    action_init();

    // ---- 头部舵机 (默认90°) ----
    sonarServo.attach(IO_Servo);
    sonarServo.write(90);
    action_wait(300);

    // ---- LiDAR 初始化 ----
    pinMode(PIN_LIDAR_MOTOR, OUTPUT);
    digitalWrite(PIN_LIDAR_MOTOR, HIGH);
    lidar_init();
    lidar_nav_reset();

    // ---- NavigateToObject 初始化 ----
    nto_reset();

    Serial.println("[就绪] 开始 LiDAR 导航到终点区\n");
}

// ============================================================
// loop
// ============================================================
void loop() {
    static MainStage lastStage = STAGE_DONE;
    if (g_stage != lastStage) {
        Serial.printf("\n>>> 进入阶段: %s <<<\n\n", stageName(g_stage));
        lastStage = g_stage;
    }

    switch (g_stage) {

    // ============================================================
    // 阶段 1: LiDAR 导航到终点区
    // ============================================================
    case STAGE_NAV: {
        lidar_update();
        if (!g_scanReady) break;
        g_scanReady = false;

        // 检查是否到达终点区
        if (lidar_isAtEndZone()) {
            Serial.println("[导航] 已到达终点区!");
            digitalWrite(PIN_LIDAR_MOTOR, LOW);
            Controller.runActionGroup(0, 1);
            action_wait(500);

            // S3 切换到 AI 文字识别模式
            s3SetMode(ESP32S3_MODE_AI);
            Serial.println("[S3] 切换到 AI 文字识别模式");

            g_stage = STAGE_ARRIVED;
            break;
        }

        // 导航一步
        NavCmd cmd = lidar_nav_toPlatform();
        if (cmd.turnAngle != 0 || cmd.forwardMm > 0) {
            action_execute(cmd);
        }
        break;
    }

    // ============================================================
    // 阶段 2: 等待裁判文字指令
    // ============================================================
    case STAGE_ARRIVED: {
        static bool  s3ModeSet  = false;
        static int   lastShape  = 0;
        static int   confirmCnt = 0;
        static bool  confirmed  = false;
        static bool  sawZero    = false;

        if (lastStage != STAGE_ARRIVED) {
            s3ModeSet  = false;
            lastShape  = 0;
            confirmCnt = 0;
            confirmed  = false;
            sawZero    = false;
        }

        if (!s3ModeSet) {
            s3SetMode(ESP32S3_MODE_AI);
            s3ModeSet = true;
            Serial.println("  [S3] 确认 AI 标签模式");
        }

        int shape = readTextFromESP32();

        if (shape == 0x11 || shape == 0x12 || shape == 0x13) {
            if (!sawZero) {
                // 还没见过 0x00, 忽略旧缓存
            } else if (shape == lastShape) {
                confirmCnt++;
            } else {
                lastShape  = shape;
                confirmCnt = 1;
            }
            if (confirmCnt >= 3 && !confirmed) {
                confirmed = true;
                Serial.printf("\n========================================\n");
                Serial.printf("  识别结果: %s\n", shapeName(shape));
                Serial.printf("========================================\n\n");

                hwsensor.asr_speak(ASR_ANNOUNCER, shape);

                // 开启雷达, 准备分类
                digitalWrite(PIN_LIDAR_MOTOR, HIGH);
                action_wait(1500);

                nto_reset();
                nto_textReceived(shape);

                g_stage = STAGE_CLASSIFY;
                break;
            }
        } else {
            sawZero    = true;
            confirmCnt = 0;
        }

        static unsigned long lastWaitPrint = 0;
        if (millis() - lastWaitPrint > 1000) {
            lastWaitPrint = millis();
            Serial.printf("  [等待文字] %s\n",
                          sawZero ? (shape ? "识别中..." : "等待指令...") : "等待S3刷新...");
        }
        break;
    }

    // ============================================================
    // 阶段 3: 终点区 3 轮文字驱动分类
    // ============================================================
    case STAGE_CLASSIFY: {
        lidar_update();

        NTO_Phase ph = nto_phase();

        if (!g_scanReady && ph != NTO_WAIT_TEXT &&
            ph != NTO_TURN_HEAD && ph != NTO_HEAD_BACK &&
            ph != NTO_DONE) break;
        if (g_scanReady) g_scanReady = false;

        static NTO_Phase lastPh = NTO_PARALLEL;

        // ---- 串口调试 ----
        int objs[3];
        int n = lidar_findObjects(objs);
        Serial.printf("[%lu] [%s]", millis(),
                      ph == NTO_WAIT_TEXT   ? "等文字" :
                      ph == NTO_CLASSIFY    ? "分类"   :
                      ph == NTO_SUMMARY     ? "总结"   :
                      ph == NTO_FIND_TARGET ? "查表"   :
                      ph == NTO_SLIDE       ? "侧滑"   :
                      ph == NTO_APPROACH    ? "逼近"   :
                      ph == NTO_VOICE       ? "播报"   :
                      ph == NTO_HEAD_BACK   ? "回正"   :
                      ph == NTO_TURN_HEAD   ? "偏头"   :
                      ph == NTO_DONE        ? "完成"   : "?");
        if (n >= 3) {
            float d0 = g_map[(objs[0]+360)%360];
            float d1 = g_map[(objs[1]+360)%360];
            float d2 = g_map[(objs[2]+360)%360];
            Serial.printf(" d=%.0f/%.0f/%.0fcm | 90°=%.0fcm",
                          d0/10, d1/10, d2/10, g_map[90]/10);
        }
        Serial.println();

        // ---- WAIT_TEXT: 读文字 (第2/3轮) ----
        if (ph == NTO_WAIT_TEXT) {
            static bool  s3ModeSet  = false;
            static int   lastShape  = 0;
            static int   confirmCnt = 0;
            static bool  confirmed  = false;
            static bool  sawZero    = false;

            if (lastPh != NTO_WAIT_TEXT) {
                s3ModeSet  = false;
                lastShape  = 0;
                confirmCnt = 0;
                confirmed  = false;
                sawZero    = false;
                digitalWrite(PIN_LIDAR_MOTOR, LOW);
                Serial.println("  [雷达] 关闭");
            }

            if (!s3ModeSet) {
                s3SetMode(ESP32S3_MODE_AI);
                s3ModeSet = true;
                Serial.println("  [S3] 切换到 AI 标签模式");
            }

            int shape = readTextFromESP32();

            if (shape == 0x11 || shape == 0x12 || shape == 0x13) {
                if (!sawZero) {
                    // 忽略旧缓存
                } else if (shape == lastShape) {
                    confirmCnt++;
                } else {
                    lastShape  = shape;
                    confirmCnt = 1;
                }
                if (confirmCnt >= 3 && !confirmed) {
                    confirmed = true;
                    digitalWrite(PIN_LIDAR_MOTOR, HIGH);
                    Serial.println("  [雷达] 开启");
                    Serial.printf("\n========================================\n");
                    Serial.printf("  识别结果: %s\n", shapeName(shape));
                    Serial.printf("========================================\n\n");
                    hwsensor.asr_speak(ASR_ANNOUNCER, shape);
                    nto_textReceived(shape);
                }
            } else {
                sawZero    = true;
                confirmCnt = 0;
            }
        }

        // ---- TURN_HEAD: 偏头到 0° ----
        static unsigned long headTurnStart = 0;
        if (ph == NTO_TURN_HEAD && lastPh != NTO_TURN_HEAD) {
            sonarServo.write(0);
            headTurnStart = millis();
            Serial.println("  [转头] 偏开 → 0°");
        }
        if (ph == NTO_TURN_HEAD && millis() - headTurnStart > 1500) {
            nto_headTurnDone();
        }

        // ---- HEAD_BACK: 回正到 90° ----
        if (ph == NTO_HEAD_BACK && lastPh != NTO_HEAD_BACK) {
            sonarServo.write(90);
            headTurnStart = millis();
            Serial.println("  [回正] → 90°");
        }
        if (ph == NTO_HEAD_BACK && millis() - headTurnStart > 1500) {
            nto_headBackDone();
        }

        // ---- SUMMARY: 从左到右播报 ----
        if (ph == NTO_SUMMARY && lastPh != NTO_SUMMARY) {
            int angles[3], shapes[3];
            nto_getObjects(angles, shapes);
            Serial.println("  [总结] 从左到右:");
            const char* names[] = {"左", "中", "右"};
            for (int i = 0; i < 3; i++) {
                Serial.printf("    %s: %s\n", names[i], shapeName(shapes[i]));
                hwsensor.asr_speak(ASR_ANNOUNCER, shapes[i] > 0 ? shapes[i] : 0x11);
                action_wait(2000);
            }
            nto_summaryDone();
        }

        // ---- VOICE: 播报当前物体 3 次 ----
        if (ph == NTO_VOICE && lastPh != NTO_VOICE) {
            int curShape = nto_currentShape();
            Serial.printf("  [语音] 播报 %s ×3\n", shapeName(curShape));
            for (int i = 0; i < 3; i++) {
                hwsensor.asr_speak(ASR_ANNOUNCER, curShape);
                action_wait(2000);
            }
        }

        lastPh = ph;

        // ---- 导航更新 ----
        NavCmd cmd = nto_update();

        if (cmd.arrived) {
            Serial.println("\n>>> 全部完成！");
            Controller.runActionGroup(0, 1);
            digitalWrite(PIN_LIDAR_MOTOR, LOW);
            g_stage = STAGE_DONE;
            break;
        }

        if (cmd.turnAngle != 0 || cmd.forwardMm > 0) {
            action_execute(cmd);
        }
        break;
    }

    // ============================================================
    // 阶段 4: 完成
    // ============================================================
    case STAGE_DONE:
        break;
    }

    delay(30);
}
