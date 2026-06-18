/*
 * lidar_navigate.cpp — 识别区导航（极简版）
 */

#include "lidar_navigate.h"
#include <math.h>

#define ROBOT_FRONT      90

// 搜索
#define SEARCH_START     10
#define SEARCH_END       170
#define OBJ_MIN_DIST     100
#define OBJ_MAX_DIST     3000

// 聚类
#define GAP_DEG          8
#define DIST_TOL         300
#define MIN_SPAN         3
#define MAX_SPAN         50
#define GROUP_MIN_SPAN   10
#define GROUP_MAX_SPAN   60

// 导航
#define FACE_OK          10
#define TARGET_MM        800
#define ARRIVE_TOL       150
#define TURN_ANGLE       15
#define MAX_FWD          300

// ============================================================
static NavPhase g_phase = NAV_FACE_OBJ;
static int  g_lastMid = -1;
static int  g_stableCnt = 0;
#define  STABLE_NEED  3
#define  STABLE_TOL   8

void lidar_nav_reset() {
    g_phase = NAV_FACE_OBJ;
    g_lastMid = -1;
    g_stableCnt = 0;
}
NavPhase lidar_nav_phase() { return g_phase; }

// ============================================================
static int norm(int a) { return (a + 360) % 360; }
static int angleDiff(int a, int b) {
    int d = a - b;
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    return d;
}

// ============================================================
int lidar_findObjects(int* out) {
    struct Cl { int s, e, n; float sa, sd; };
    Cl cs[30]; int nc = 0;
    Cl cur; bool in = false;

    for (int i = SEARCH_START; i <= SEARCH_END; i++) {
        float d = g_map[i];
        bool ok = (d > OBJ_MIN_DIST && d < OBJ_MAX_DIST);
        if (ok && !in) {
            in = true; cur.s = cur.e = i; cur.n = 1; cur.sa = i; cur.sd = d;
        } else if (ok && in) {
            if ((i - cur.e) <= GAP_DEG && fabsf(d - g_map[cur.e]) < DIST_TOL) {
                cur.e = i; cur.n++; cur.sa += i; cur.sd += d;
            } else {
                if (cur.n >= MIN_SPAN && (cur.e - cur.s + 1) <= MAX_SPAN && nc < 30)
                    cs[nc++] = cur;
                cur.s = cur.e = i; cur.n = 1; cur.sa = i; cur.sd = d;
            }
        } else if (!ok && in) {
            if (cur.n >= MIN_SPAN && (cur.e - cur.s + 1) <= MAX_SPAN && nc < 30)
                cs[nc++] = cur;
            in = false;
        }
    }
    if (in && cur.n >= MIN_SPAN && (cur.e - cur.s + 1) <= MAX_SPAN && nc < 30)
        cs[nc++] = cur;

    if (nc < 3) return 0;

    // 计算中心 + 按距离排序
    int cen[30]; float ad[30];
    for (int i = 0; i < nc; i++) {
        cen[i] = (int)(cs[i].sa / cs[i].n + 0.5f);
        ad[i] = cs[i].sd / cs[i].n;
    }
    for (int i = 0; i < nc-1; i++)
        for (int j = i+1; j < nc; j++)
            if (ad[j] < ad[i]) {
                float t = ad[i]; ad[i] = ad[j]; ad[j] = t;
                int c = cen[i]; cen[i] = cen[j]; cen[j] = c;
            }

    // 取最近3个，按角度排序
    int n = (nc < 3) ? nc : 3;
    for (int i = 0; i < n-1; i++)
        for (int j = i+1; j < n; j++)
            if (cen[j] < cen[i]) { int t = cen[i]; cen[i] = cen[j]; cen[j] = t; }
    for (int i = 0; i < n; i++) out[i] = cen[i];
    return n;
}

// ============================================================
NavCmd lidar_nav_toPlatform() {
    NavCmd cmd = {0,0,false,false};

    int objs[3];
    if (lidar_findObjects(objs) < 3) return cmd;

    int M = objs[1];
    float midDist = g_map[M];
    if (midDist <= 0) return cmd;

    // ---- 动作前稳定：连续 STABLE_NEED 次 mid 变化 < STABLE_TOL ----
    int m = norm(M);
    if (g_lastMid >= 0 && abs(m - g_lastMid) <= STABLE_TOL) {
        g_stableCnt++;
    } else {
        g_stableCnt = 0;
    }
    g_lastMid = m;
    if (g_stableCnt < STABLE_NEED) {
        Serial.printf("  [稳定] mid=%d° %d/%d\n", m, g_stableCnt, STABLE_NEED);
        return cmd;
    }

    // ---- 导航 ----
    switch (g_phase) {
    case NAV_FACE_OBJ: {
        int err = angleDiff(M, ROBOT_FRONT);
        if (abs(err) <= FACE_OK) {
            g_phase = NAV_APPROACH;
            g_stableCnt = 0;  // 换阶段，重新稳定
            return cmd;
        }
        cmd.turnAngle = (err > 0) ? -TURN_ANGLE : TURN_ANGLE;
        g_stableCnt = 0;  // 动作后重新稳定
        return cmd;
    }
    case NAV_APPROACH: {
        int err = angleDiff(M, ROBOT_FRONT);
        if (abs(err) > FACE_OK + 15) {
            g_phase = NAV_FACE_OBJ;
            g_stableCnt = 0;
            return cmd;
        }
        if (abs(err) > FACE_OK) {
            cmd.turnAngle = (err > 0) ? -TURN_ANGLE : TURN_ANGLE;
            cmd.smallTurn = true;
            g_stableCnt = 0;
            return cmd;
        }
        float step = midDist - TARGET_MM;
        if (fabsf(step) <= ARRIVE_TOL) {
            g_phase = NAV_ARRIVED; cmd.arrived = true;
            return cmd;
        }
        step *= 0.6f;
        if (step > MAX_FWD) step = MAX_FWD;
        if (step < 30) step = 30;
        cmd.forwardMm = (int)step;
        g_stableCnt = 0;
        return cmd;
    }
    case NAV_ARRIVED:
        cmd.arrived = true;
        return cmd;
    }
    return cmd;
}
