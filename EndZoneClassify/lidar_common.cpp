/*
 * lidar_common.cpp — LiDAR 初始化 + 数据采集
 * B 不需要改这个文件
 */

#include "lidar_common.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

float   g_map[360];
uint8_t g_qual[360];
bool    g_scanReady = false;
RPLidar lidar;

static int g_lastAngle = -1;

// 喂狗延时 — 防止长延时触发 ESP32 看门狗复位
static void safeWait(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        delay(10);
        TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
        TIMERG1.wdt_feed = 1;
        TIMERG1.wdt_wprotect = 0;
    }
}

void lidar_init() {
    // 启动电机
    pinMode(PIN_LIDAR_MOTOR, OUTPUT);
    digitalWrite(PIN_LIDAR_MOTOR, HIGH);
    safeWait(2000);

    // 打开 Serial1
    Serial1.begin(115200, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);
    safeWait(100);
    while (Serial1.available()) Serial1.read();

    lidar.begin(Serial1);
    // RPLidar::begin() 不再重置串口引脚，直接可用

    // 验证连接
    rplidar_response_device_info_t info;
    if (!IS_OK(lidar.getDeviceInfo(info, 1000))) {
        Serial.println("[FATAL] LiDAR 连接失败");
        digitalWrite(PIN_LIDAR_MOTOR, LOW);
        while (1) safeWait(1000);
    }

    // 开始扫描
    if (!IS_OK(lidar.startScan())) {
        Serial.println("[FATAL] 扫描启动失败");
        digitalWrite(PIN_LIDAR_MOTOR, LOW);
        while (1) safeWait(1000);
    }

    safeWait(500);
    Serial.println("[OK] LiDAR 就绪");
}

void lidar_update() {
    if (!IS_OK(lidar.waitPoint())) return;

    float   dist  = lidar.getCurrentPoint().distance;
    float   angle = lidar.getCurrentPoint().angle;
    uint8_t qual  = lidar.getCurrentPoint().quality;

    int idx = (int)(angle + 0.5f) % 360;

    // 检测新一圈（角度从大跳小）
    if (g_lastAngle > 350 && idx < 10) {
        g_scanReady = true;
    }
    g_lastAngle = idx;

    if (qual > 0 && dist > 0) {
        g_map[idx]  = dist;
        g_qual[idx] = qual;
    }
}

void lidar_streamScan() {
    int valid = 0;
    for (int i = 0; i < 360; i++) {
        if (g_map[i] > 0) valid++;
    }

    Serial.printf("SCAN %lu %d\n", millis(), valid);
    for (int i = 0; i < 360; i++) {
        if (g_map[i] > 0) {
            Serial.printf("%d %.0f %d\n", i, g_map[i], g_qual[i]);
        }
    }
    Serial.println("END");
}
