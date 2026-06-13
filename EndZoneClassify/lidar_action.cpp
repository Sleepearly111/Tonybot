/*
 * lidar_action.cpp — 舵机动作执行
 */

#include "lidar_action.h"
#include "base_config.h"
#include "LobotServoController.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// 动作组编号
#define ACT_STAND         0
#define ACT_TURN_LEFT_S   34   // 小幅左转
#define ACT_TURN_RIGHT_S  35   // 小幅右转
#define ACT_TURN_LEFT     23
#define ACT_TURN_RIGHT    24
#define ACT_FORWARD       25

// 换算参数（实测调整）
#define DEG_PER_ACT     15
#define MM_PER_ACT      80
#define MAX_TURN_TIMES  4
#define MAX_FWD_TIMES   4

// 等待时间
#define TURN_WAIT_MS    800
#define FWD_WAIT_MS     800
#define SETTLE_MS       1500

static LobotServoController* g_ctrl = nullptr;

// ============================================================
void action_init() {
    Serial2.begin(9600, SERIAL_8N1, IO_BaseRX, IO_BaseTX);
    action_wait(100);
    g_ctrl = new LobotServoController(Serial2);
    action_stand();
    action_wait(2000);
}

void action_wait(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        delay(10);
        TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
        TIMERG1.wdt_feed = 1;
        TIMERG1.wdt_wprotect = 0;
    }
}

void action_stand() {
    if (g_ctrl) g_ctrl->runActionGroup(ACT_STAND, 1);
}

void action_execute(const NavCmd& cmd) {
    if (!g_ctrl) return;

    if (cmd.arrived) {
        g_ctrl->runActionGroup(ACT_STAND, 1);
        return;
    }

    if (cmd.turnAngle != 0) {
        int g;
        if (cmd.smallTurn) {
            g = (cmd.turnAngle > 0) ? ACT_TURN_LEFT_S : ACT_TURN_RIGHT_S;
            Serial.printf("  → 舵机: 小%s × 1\n",
                          cmd.turnAngle > 0 ? "左转" : "右转");
        } else {
            g = (cmd.turnAngle > 0) ? ACT_TURN_LEFT : ACT_TURN_RIGHT;
            Serial.printf("  → 舵机: %s × 1\n",
                          cmd.turnAngle > 0 ? "左转" : "右转");
        }
        g_ctrl->runActionGroup(g, 1);
        action_wait(TURN_WAIT_MS);

    } else if (cmd.forwardMm > 0) {
        int times = cmd.forwardMm / MM_PER_ACT;
        if (times < 1) times = 1;
        if (times > MAX_FWD_TIMES) times = MAX_FWD_TIMES;

        Serial.printf("  → 舵机: 前进 × %d\n", times);
        g_ctrl->runActionGroup(ACT_FORWARD, times);
        for (int i = 0; i < times; i++) action_wait(FWD_WAIT_MS);
    }

    action_wait(SETTLE_MS);
}
