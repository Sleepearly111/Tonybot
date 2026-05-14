#include "iic_data_send.hpp"
#include "color_detection.hpp"
#include "Wire.h"

#define I2C_SLAVE_ADDRESS 0x52

static QueueHandle_t xQueueResultI = NULL;
static QueueHandle_t xQueueResultO = NULL;

static const char *TAG = "iic_data_send";
static const int sdaPin = 47;
static const int sclPin = 48;
static const uint32_t i2cFrequency = 100000;

static send_color_data_t color_data[5];

// 低通滤波用的历史值
static float filter_cx = 0;
static float filter_cy = 0;
static float filter_w  = 0;
static float filter_h  = 0;
static const float FILTER_ALPHA = 0.3f; // 滤波系数（越小越平滑）

static uint8_t rec = 0xFF;
static uint8_t send_data[4] = {0};

/* I2C 接收回调：主机发送颜色模式切换命令 */
static void iic_receive(int len)
{
    while (Wire.available()) {
        rec = Wire.read();
        // 命令码 0x01 → 红色模式
        if (rec == 0x01) {
            g_color_mode = 0;
        }
        // 命令码 0x02 → 蓝色模式
        else if (rec == 0x02) {
            g_color_mode = 1;
        }
    }
}

/* I2C 请求回调：根据当前颜色模式发送对应色块数据 */
static void iic_request()
{
    int idx = -1;

    // 根据 g_color_mode 选择要发送的颜色数据索引
    if (g_color_mode == 0) {
        idx = 0;  // 红色 → color_data[0]
    } else {
        idx = 3;  // 蓝色 → color_data[3]
    }

    send_data[0] = color_data[idx].center_x;
    send_data[1] = color_data[idx].center_y;
    send_data[2] = color_data[idx].width;
    send_data[3] = color_data[idx].length;

    Wire.slaveWrite(send_data, sizeof(send_data));
}

static void task_process_handler(void *arg)
{
    /* IIC 从机初始化 */
    Wire.begin((uint8_t)I2C_SLAVE_ADDRESS, sdaPin, sclPin, i2cFrequency);
    /* 注册 I2C 从机回调 */
    Wire.onReceive(iic_receive);
    Wire.onRequest(iic_request);

    int frame_count = 0;

    while (true) {
        if (xQueueReceive(xQueueResultI, &color_data, portMAX_DELAY)) {

            // 低通滤波：对发送给主控的坐标进行平滑，减少靠近时的抖动
            int idx = (g_color_mode == 0) ? 0 : 3;
            float raw_cx = (float)color_data[idx].center_x;
            float raw_cy = (float)color_data[idx].center_y;
            float raw_w  = (float)color_data[idx].width;
            float raw_h  = (float)color_data[idx].length;

            if (raw_cx > 0) { // 有检测到目标时才滤波
                filter_cx = FILTER_ALPHA * raw_cx + (1.0f - FILTER_ALPHA) * filter_cx;
                filter_cy = FILTER_ALPHA * raw_cy + (1.0f - FILTER_ALPHA) * filter_cy;
                filter_w  = FILTER_ALPHA * raw_w  + (1.0f - FILTER_ALPHA) * filter_w;
                filter_h  = FILTER_ALPHA * raw_h  + (1.0f - FILTER_ALPHA) * filter_h;

                color_data[idx].center_x = (uint8_t)filter_cx;
                color_data[idx].center_y = (uint8_t)filter_cy;
                color_data[idx].width    = (uint8_t)filter_w;
                color_data[idx].length   = (uint8_t)filter_h;
            } else {
                // 目标丢失，重置滤波器
                filter_cx = 0;
                filter_cy = 0;
                filter_w  = 0;
                filter_h  = 0;
            }

            // 调试打印（每 10 帧打印一次）
            if (++frame_count >= 10) {
                frame_count = 0;
                printf("[DEBUG] mode=%d, cx=%d, cy=%d, w=%d, h=%d\n",
                       g_color_mode,
                       (int)color_data[idx].center_x,
                       (int)color_data[idx].center_y,
                       (int)color_data[idx].width,
                       (int)color_data[idx].length);

                if (color_data[idx].center_x == 0) {
                    printf("[DEBUG] target lost\n");
                }
            }
        }
    }
}

void register_iic_data_send(const QueueHandle_t result_i,
                            const QueueHandle_t result_o)
{
    xQueueResultI = result_i;
    xQueueResultO = result_o;

    xTaskCreatePinnedToCore(task_process_handler, TAG, 4 * 1024, NULL, 5, NULL, 1);
}
