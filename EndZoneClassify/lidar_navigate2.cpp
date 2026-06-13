/*
 * lidar_navigate2.cpp — 极简导航
 * 1. 等mid稳定 → 2. 转向对准 → 3. 直走 → 4. 距离到就停
 */
#include "lidar_navigate2.h"
#include <math.h>

#define ROBOT_FRONT  90
#define FACE_OK      10
#define TARGET_MM    800
#define ARRIVE_TOL   150
#define TURN_DEG     15
#define MAX_FWD      300
#define STABLE_NEED  2
#define STABLE_TOL   8

static Nav2Phase g_phase = N2_STABLE;
static int g_lastMid = -1;
static int g_stableCnt = 0;

void nav2_reset() {
    g_phase = N2_STABLE;
    g_lastMid = -1;
    g_stableCnt = 0;
}
Nav2Phase nav2_phase() { return g_phase; }

static int norm(int a) { return (a + 360) % 360; }
static int angleDiff(int a, int b) {
    int d = a - b;
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    return d;
}

NavCmd nav2_update() {
    NavCmd cmd = {0, 0, false, false};

    int objs[3];
    if (lidar_findObjects(objs) < 3) return cmd;

    int M = objs[1];
    float midDist = g_map[M];
    if (midDist <= 0) return cmd;

    int m = norm(M);

    // ---- 稳定（GO阶段跳过）----
    if (g_phase != N2_GO && g_phase != N2_ARRIVED) {
        if (g_lastMid >= 0 && abs(m - g_lastMid) <= STABLE_TOL)
            g_stableCnt++;
        else
            g_stableCnt = 0;
        g_lastMid = m;

        if (g_stableCnt < STABLE_NEED) {
            Serial.printf("  [稳定] mid=%d° %d/%d\n", m, g_stableCnt, STABLE_NEED);
            return cmd;
        }
    }

    // ---- 导航 ----
    switch (g_phase) {
    case N2_STABLE:
        g_phase = N2_TURN;
        g_stableCnt = 0;
        return cmd;

    case N2_TURN: {
        int err = angleDiff(M, ROBOT_FRONT);
        if (abs(err) <= FACE_OK) {
            g_phase = N2_GO;
            g_stableCnt = 0;
            return cmd;
        }
        cmd.turnAngle = (err > 0) ? -TURN_DEG : TURN_DEG;
        g_stableCnt = 0;
        return cmd;
    }

    case N2_GO: {
        float remain = midDist - TARGET_MM;
        if (fabsf(remain) <= ARRIVE_TOL) {
            g_phase = N2_ARRIVED;
            cmd.arrived = true;
            return cmd;
        }
        // >130cm走3步，否则走1步
        cmd.forwardMm = (midDist > 1300) ? 240 : 80;
        return cmd;
    }

    case N2_ARRIVED:
        cmd.arrived = true;
        return cmd;
    }
    return cmd;
}
