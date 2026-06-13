/*
 * LiDARTest.ino — navigate2 导航
 */

#include "lidar_common.h"
#include "lidar_navigate2.h"
#include "lidar_classify.h"
#include "lidar_action.h"

static const char* phaseName(Nav2Phase p) {
    switch (p) {
        case N2_STABLE:   return "稳定";
        case N2_TURN:     return "转正";
        case N2_GO:       return "直走";
        case N2_ARRIVED:  return "到达";
        default: return "?";
    }
}

static const char* shapeName(int s) {
    switch (s) { case 1: return "球体"; case 2: return "正方体"; case 3: return "圆柱体"; default: return "?"; }
}

void setup() {
    Serial.begin(115200);
    action_wait(500);
    Serial.println("\n====== navigate2 导航测试 ======");
    action_init();
    Serial.println("[初始化] 雷达电机...");
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    action_wait(2000);
    lidar_init();
    nav2_reset();
    Serial.println("[就绪]\n");
}

void loop() {
    lidar_update();
    if (!g_scanReady) return;
    g_scanReady = false;

    int objs[3];
    int n = lidar_findObjects(objs);
    Nav2Phase ph = nav2_phase();

    // 到达
    if (ph == N2_ARRIVED) {
        static int lastShapes[3] = {-1,-1,-1};
        static int classStable = 0;
        if (n >= 3) {
            int shapes[3];
            classifyThree(objs, shapes);
            bool same = true;
            for (int i = 0; i < 3; i++)
                if (shapes[i] != lastShapes[i]) { same = false; break; }
            if (same) {
                classStable++;
                if (classStable >= 3) {
                    Serial.println("\n>>> 到达终点区！");
                    for (int i = 0; i < 3; i++) {
                        ShapeInfo info;
                        classifyObject(objs[i], &info);
                        Serial.printf("  [%d] %d° %s\n", i+1, (objs[i]+360)%360, shapeName(shapes[i]));
                    }
                    action_stand();
                    while (1) action_wait(1000);
                }
            } else {
                classStable = 0;
                for (int i = 0; i < 3; i++) lastShapes[i] = shapes[i];
            }
        }
    }

    NavCmd cmd = nav2_update();

    Serial.printf("[%lu] [%s] [%d° %d° %d°] mid=%.0fcm\n",
                  millis(), phaseName(ph),
                  (objs[0]+360)%360, (objs[1]+360)%360, (objs[2]+360)%360,
                  n>=3 ? g_map[(objs[1]+360)%360]/10.0f : 0);

    if (!cmd.arrived) action_execute(cmd);
}
