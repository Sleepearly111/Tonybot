/*
 * main.cpp — ESP32-S3 视觉协处理器状态机
 *
 * 模式由 ESP32 通过 I2C 寄存器 0x30 控制:
 *   0x01 = 颜色追踪模式 (Task1: 绕柱, ObjectFollower/CirclePillar)
 *   0x02 = AI 标签识别模式 (Task2+3: 终点区读汉字标签)
 *
 * I2C 寄存器:
 *   0x00 = 红色色块 (x, y, w, h) — 4 bytes
 *   0x01 = 蓝色色块 (x, y, w, h) — 4 bytes
 *   0x10 = AI 标签结果 (0x11=球体, 0x12=正方体, 0x13=圆柱体)
 *   0x20 = AI 形状结果 (1=球体, 2=正方体, 3=圆柱体)
 *   0x30 = 模式寄存器 (写入: 0x01/0x02)
 */

#include <Arduino.h>
#include <Wire.h>
#include "esp_camera.h"
#include "camera_setting.h"
#include "color_detector.hpp"
#include "VisionAI.h"

// ============================================================
// 颜色阈值 (HSV, RGB565)
// ============================================================
#define COLOR_RED    0x00F8
#define COLOR_BLUE   0x1F00

// 颜色在 std_color_info 中的索引
#define IDX_RED   0
#define IDX_BLUE  3

// ============================================================
// I2C 寄存器数据
// ============================================================
#define I2C_SLAVE_ADDR  0x52
#define I2C_SDA         47
#define I2C_SCL         48

// 颜色数据: [0]=red, [3]=blue (兼容原 color_detection 的索引)
struct ColorBlob {
    uint8_t x, y, width, length;
};
static ColorBlob g_color_data[5];  // 5个颜色槽, 实际用 [0]=red, [3]=blue

// AI 结果
static uint8_t g_ai_label = 0x00;  // 0x11=球体 0x12=正方体 0x13=圆柱体
static uint8_t g_ai_shape = 0x00;  // 1=球体 2=正方体 3=圆柱体

// 当前模式 (ESP32 通过 I2C 写入)
volatile uint8_t g_mode = 0x00;     // 0x00=idle, 0x01=color, 0x02=AI

// I2C 状态
static uint8_t g_i2c_reg = 0x00;   // 当前寄存器指针
static uint8_t g_send_buf[4];      // 发送缓冲

// ColorDetector 实例
static ColorDetector *g_red_detector = nullptr;
static ColorDetector *g_blue_detector = nullptr;

// ============================================================
// I2C 回调
// ============================================================
static void onI2CReceive(int len) {
    if (Wire.available()) {
        g_i2c_reg = Wire.read();  // 寄存器地址

        // 如果是写模式寄存器 (0x30 + value)
        if (g_i2c_reg == 0x30 && Wire.available()) {
            uint8_t newMode = Wire.read();
            if (newMode != g_mode) {
                g_mode = newMode;
                Serial.printf("[S3] 模式切换 → 0x%02X (%s)\n",
                              g_mode,
                              g_mode == 0x01 ? "颜色追踪" :
                              g_mode == 0x02 ? "AI标签" : "空闲");
            }
        }
    }
    // 排空剩余字节
    while (Wire.available()) { Wire.read(); }
}

static void onI2CRequest() {
    switch (g_i2c_reg) {
        case 0x00:  // 红色色块
            g_send_buf[0] = g_color_data[IDX_RED].x;
            g_send_buf[1] = g_color_data[IDX_RED].y;
            g_send_buf[2] = g_color_data[IDX_RED].width;
            g_send_buf[3] = g_color_data[IDX_RED].length;
            break;
        case 0x01:  // 蓝色色块
            g_send_buf[0] = g_color_data[IDX_BLUE].x;
            g_send_buf[1] = g_color_data[IDX_BLUE].y;
            g_send_buf[2] = g_color_data[IDX_BLUE].width;
            g_send_buf[3] = g_color_data[IDX_BLUE].length;
            break;
        case 0x10:  // AI 标签
            g_send_buf[0] = g_ai_label;
            g_send_buf[1] = 0; g_send_buf[2] = 0; g_send_buf[3] = 0;
            break;
        case 0x20:  // AI 形状
            g_send_buf[0] = g_ai_shape;
            g_send_buf[1] = 0; g_send_buf[2] = 0; g_send_buf[3] = 0;
            break;
        default:
            memset(g_send_buf, 0, 4);
            break;
    }
    Wire.slaveWrite(g_send_buf, 4);
}

// ============================================================
// 颜色检测辅助
// ============================================================
static std::vector<color_info_t> makeRedThreshold() {
    return {{{151, 15, 70, 255, 90, 255}, 64, "red"}};
}
static std::vector<color_info_t> makeBlueThreshold() {
    return {{{97, 117, 70, 255, 90, 255}, 64, "blue"}};
}

static void extractBlob(std::vector<color_detect_result_t> &results, ColorBlob &blob) {
    if (results.empty()) {
        blob.x = 0; blob.y = 0; blob.width = 0; blob.length = 0;
        return;
    }
    int maxIdx = 0, maxArea = 0;
    for (int i = 0; i < (int)results.size(); i++) {
        if (results[i].area > maxArea) { maxArea = results[i].area; maxIdx = i; }
    }
    blob.x      = (uint8_t)results[maxIdx].center[0];
    blob.y      = (uint8_t)results[maxIdx].center[1];
    blob.width  = (uint8_t)(results[maxIdx].box[2] - results[maxIdx].box[0]);
    blob.length = (uint8_t)(results[maxIdx].box[3] - results[maxIdx].box[1]);
}

// ============================================================
// 颜色追踪模式 — 检测红蓝双色
// ============================================================
static void runColorTracking(camera_fb_t *fb) {
    // ColorDetector 直接接收 RGB565 帧
    auto &redResults  = g_red_detector->detect(
        (uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
    auto &blueResults = g_blue_detector->detect(
        (uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});

    if (!redResults.empty())
        extractBlob(redResults[0], g_color_data[IDX_RED]);
    else
        g_color_data[IDX_RED] = {0,0,0,0};

    if (!blueResults.empty())
        extractBlob(blueResults[0], g_color_data[IDX_BLUE]);
    else
        g_color_data[IDX_BLUE] = {0,0,0,0};
}

// ============================================================
// setup
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[S3] 视觉协处理器启动");

    // --- 相机 ---
    Serial.println("[S3] 初始化相机...");
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0  = Y2_GPIO_NUM;
    config.pin_d1  = Y3_GPIO_NUM;
    config.pin_d2  = Y4_GPIO_NUM;
    config.pin_d3  = Y5_GPIO_NUM;
    config.pin_d4  = Y6_GPIO_NUM;
    config.pin_d5  = Y7_GPIO_NUM;
    config.pin_d6  = Y8_GPIO_NUM;
    config.pin_d7  = Y9_GPIO_NUM;
    config.pin_xclk  = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.frame_size   = FRAMESIZE_240X240;
    config.pixel_format = PIXFORMAT_RGB565;
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 16;
    config.fb_count     = 4;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[S3] 相机初始化失败: 0x%x\n", err);
        return;
    }
    Serial.println("[S3] 相机就绪");

    // --- 颜色检测器 ---
    g_red_detector  = new ColorDetector();
    g_blue_detector = new ColorDetector();
    g_red_detector->register_color(makeRedThreshold()[0].color_thresh,
                                    makeRedThreshold()[0].area_thresh, "red");
    g_blue_detector->register_color(makeBlueThreshold()[0].color_thresh,
                                     makeBlueThreshold()[0].area_thresh, "blue");
    Serial.println("[S3] 颜色检测器就绪");

    // --- I2C 从机 ---
    Wire.begin(I2C_SLAVE_ADDR, I2C_SDA, I2C_SCL, 100000);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    Serial.println("[S3] I2C 从机就绪 (0x52)");

    Serial.println("[S3] 就绪, 等待模式指令...\n");
}

// ============================================================
// loop
// ============================================================
void loop() {
    switch (g_mode) {

        // ========== 颜色追踪模式 ==========
        case 0x01: {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) { vTaskDelay(5); break; }

            runColorTracking(fb);
            esp_camera_fb_return(fb);

            // 调试输出
            static unsigned long lastPrint = 0;
            if (millis() - lastPrint > 500) {
                lastPrint = millis();
                Serial.printf("[S3-颜色] R:(%d,%d w=%d) B:(%d,%d w=%d)\n",
                              g_color_data[IDX_RED].x, g_color_data[IDX_RED].y,
                              g_color_data[IDX_RED].width,
                              g_color_data[IDX_BLUE].x, g_color_data[IDX_BLUE].y,
                              g_color_data[IDX_BLUE].width);
            }
            break;
        }

        // ========== AI 标签识别模式 ==========
        case 0x02: {
            int label_id = run_ai_label_recognition();

            if (label_id > 0) {
                g_ai_label = 0x10 + label_id;  // 1→0x11, 2→0x12, 3→0x13
                g_ai_shape = label_id;           // 1=球体, 2=正方体, 3=圆柱体
                Serial.printf("[S3-AI] 识别结果: %s (label=0x%02X shape=%d)\n",
                              label_id == 1 ? "球体" :
                              label_id == 2 ? "正方体" :
                              label_id == 3 ? "圆柱体" : "?",
                              g_ai_label, g_ai_shape);
            } else {
                g_ai_label = 0x00;
                g_ai_shape = 0x00;
            }
            vTaskDelay(50 / portTICK_PERIOD_MS);
            break;
        }

        // ========== 空闲 ==========
        default:
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
    }
}
