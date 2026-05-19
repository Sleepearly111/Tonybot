#include "VisionAI.h"
#include <Tonybot_OCR_Vision_inferencing.h> 
#include "esp_camera.h"                 
#include "freertos/FreeRTOS.h"   
#include "freertos/queue.h"      

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           240
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

static uint8_t *snapshot_buf = nullptr;
extern QueueHandle_t xQueueAIFrame; 

// --- 数据转换函数：绝地修复打包格式 ---
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    float step_x = (float)EI_CAMERA_RAW_FRAME_BUFFER_COLS / EI_CLASSIFIER_INPUT_WIDTH;
    float step_y = (float)EI_CAMERA_RAW_FRAME_BUFFER_ROWS / EI_CLASSIFIER_INPUT_HEIGHT;

    for (size_t i = 0; i < length; i++) {
        int current_offset = offset + i;
        int target_x_ai = current_offset % EI_CLASSIFIER_INPUT_WIDTH;
        int target_y_ai = current_offset / EI_CLASSIFIER_INPUT_WIDTH;

        int src_x = (int)(target_x_ai * step_x);
        int src_y = (int)(target_y_ai * step_y);

        size_t pixel_ix = (src_y * EI_CAMERA_RAW_FRAME_BUFFER_COLS + src_x) * 3;

        uint8_t r = snapshot_buf[pixel_ix];
        uint8_t g = snapshot_buf[pixel_ix + 1];
        uint8_t b = snapshot_buf[pixel_ix + 2];

        // 【真凶伏法】：Edge Impulse 的底层 DSP 模块要求输入必须是 0x00RRGGBB 的 32位整型格式（强转为 float）
        // 之前我们传了 0.0 到 1.0 的小数，在它眼里等于 0x000000（纯黑），所以 AI 彻底瞎了！
        uint32_t pixel = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
        out_ptr[i] = (float)pixel; 
    }
    return 0;
}

int run_ai_label_recognition() {
    if (snapshot_buf == nullptr) {
        snapshot_buf = (uint8_t*)ps_malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3);
        if (snapshot_buf == nullptr) return 0;
    }

    camera_fb_t *fb = nullptr;
    camera_fb_t *stale_fb = nullptr;
    
    // 狂暴排空积压帧
    while (xQueueReceive(xQueueAIFrame, &stale_fb, 0) == pdTRUE) {
        esp_camera_fb_return(stale_fb); 
    }

    // 等待最新鲜的照片
    if (xQueueReceive(xQueueAIFrame, &fb, portMAX_DELAY) != pdTRUE) {
        Serial.println("ERR: 摄像头队列死锁，拿不到照片");
        return 0;
    }

    if (fb->width != 240 || fb->height != 240) {
        esp_camera_fb_return(fb);
        return 0;
    }

    fmt2rgb888(fb->buf, fb->len, fb->format, snapshot_buf);
    esp_camera_fb_return(fb); 

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false); 
    if (err != EI_IMPULSE_OK) return 0;

    // 4. 模拟显示屏（解包 Edge Impulse 的大整数格式）
    Serial.println("\n--- AI 实时视觉预览 ---");
    for (int y = 0; y < EI_CLASSIFIER_INPUT_HEIGHT; y += 4) { 
        for (int x = 0; x < EI_CLASSIFIER_INPUT_WIDTH; x += 2) {
            float pixel_val;
            ei_camera_get_data(y * EI_CLASSIFIER_INPUT_WIDTH + x, 1, &pixel_val);
            
            // 把 Edge Impulse 的格式解包回灰度用来显示
            uint32_t p = (uint32_t)pixel_val;
            uint8_t r = (p >> 16) & 0xFF;
            uint8_t g = (p >> 8) & 0xFF;
            uint8_t b = p & 0xFF;
            float gray = (r * 0.299f + g * 0.587f + b * 0.114f) / 255.0f;

            if (gray > 0.5f) Serial.print("."); else Serial.print("#"); 
        }
        Serial.println();
    }

    int best_class = 0; 
    float max_score = 0.0;
    const char* winner_label = "none";

    Serial.print("AI 脑内实时打分 -> ");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float val = result.classification[i].value;
        const char* label = ei_classifier_inferencing_categories[i];
        Serial.printf("[%s: %.2f] ", label, val);

        if (val > max_score) {
            max_score = val;
            winner_label = label;
        }
    }

    if (max_score > 0.7f && strcmp(winner_label, "background") != 0) {
        if (strcmp(winner_label, "sphere") == 0)   best_class = 1; 
        else if (strcmp(winner_label, "cube") == 0)     best_class = 2; 
        else if (strcmp(winner_label, "Cylinder") == 0) best_class = 3; 
    } else {
        best_class = 0; 
    }

    Serial.printf(" | 最终决策: %s (ID: %d)\n", winner_label, best_class);
    return best_class;
}