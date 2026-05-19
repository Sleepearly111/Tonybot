#include <Arduino.h>
#include "camera_setting.h"
#include "iic_data_send.hpp"  
#include "VisionAI.h"


QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueColorResult = NULL;

void setup() {
    Serial.begin(115200); 
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));
    xQueueColorResult = xQueueCreate(1, sizeof(send_color_data_t) * 5);

    register_camera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 4, xQueueAIFrame);
    register_iic_data_send(xQueueColorResult, NULL);
    
    Serial.println(">> [S3 Brain] Communication Mock Mode Online.");
}

void loop() {
    // 1. 唤醒真正的 AI 大脑！
    int label_id = run_ai_label_recognition();
    
    // 把结果塞进 I2C 寄存器缓存
    // 2. 将真实结果更新到 I2C 寄存器缓存 (只处理有效识别，0 不发送)
    uint8_t tx_label = 0x00;
    if (label_id > 0) {
        tx_label = 0x10 + label_id; // 1->0x11, 2->0x12, 3->0x13
    }
    
    update_ai_iic_data(tx_label, 0x00);

    vTaskDelay(100 / portTICK_PERIOD_MS); 
}