#include "iic_data_send.hpp"
#include "Wire.h"

#define I2C_SLAVE_ADDRESS 0x52

static QueueHandle_t xQueueResultI = NULL;
static QueueHandle_t xQueueResultO = NULL;

static const char *TAG = "iic_data_send";
static const int sdaPin = 47;
static const int sclPin = 48;
static const uint32_t i2cFrequency = 100000;

static send_color_data_t color_data[5];

// --- 【展赫新增】：AI 识别结果全局缓存 ---
static uint8_t g_ai_label_id = 0x00; 
static uint8_t g_ai_shape_id = 0x00;

static uint8_t rec = 0xFF;
static uint8_t send_data[4] = {0};

// S3 工作模式 (ESP32 通过 I2C 寄存器 0x30 写入)
volatile uint8_t g_s3_mode = 0x00;  // 0x00=idle, 0x01=颜色追踪, 0x02=AI标签

// 【新增】：暴露给主程序的接口，用来更新 AI 识别结果
void update_ai_iic_data(uint8_t label, uint8_t shape) {
    g_ai_label_id = label;
    g_ai_shape_id = shape;
}

static void iic_receive(int len){
  // 读取第一个字节 → 寄存器地址
  if (Wire.available()) {
    rec = Wire.read();
  }
  // 如果还有第二个字节 → 寄存器值 (用于写寄存器, 如模式切换)
  if (Wire.available() && rec == 0x30) {
    g_s3_mode = Wire.read();
  }
  // 清空剩余
  while (Wire.available()) { Wire.read(); }
}

static void iic_request()
{
  /* 红色色块数据 */
  if(rec == 0x00) 
  {
    send_data[0] = color_data[0].center_x;
    send_data[1] = color_data[0].center_y;
    send_data[2] = color_data[0].width;
    send_data[3] = color_data[0].length;
  }
  /* 绿色色块数据 */
  else if(rec == 0x01)
  {
    send_data[0] = color_data[2].center_x;
    send_data[1] = color_data[2].center_y;
    send_data[2] = color_data[2].width;
    send_data[3] = color_data[2].length;
  }
  /* 蓝色色块数据 */
  else if(rec == 0x02)
  {
    send_data[0] = color_data[3].center_x;
    send_data[1] = color_data[3].center_y;
    send_data[2] = color_data[3].width;
    send_data[3] = color_data[3].length;
  }
  // --- 【核心修改】：新增 AI 寄存器读取响应 ---
    else if(rec == 0x10) { // 主控请求标签结果 (0x10)
        send_data[0] = g_ai_label_id; 
        send_data[1] = 0x00; send_data[2] = 0x00; send_data[3] = 0x00;
    }
    else if(rec == 0x20) { // 主控请求形状结果 (0x20)
        send_data[0] = g_ai_shape_id;
        send_data[1] = 0x00; send_data[2] = 0x00; send_data[3] = 0x00;
    }

  /* 发送色块数据 */
  Wire.slaveWrite(send_data, sizeof(send_data));

}

static void task_process_handler(void *arg)
{
  /* IIC初始化 */
  Wire.begin((uint8_t)I2C_SLAVE_ADDRESS, sdaPin, sclPin, i2cFrequency);
  /* 注册接收数据的回调函数 */
  Wire.onReceive(iic_receive);
  /* 注册请求数据的回调函数 */
  Wire.onRequest(iic_request);

  while (true)
  {

    if (xQueueReceive(xQueueResultI, &color_data, portMAX_DELAY))
    {
    //  switch(rec){
    //   case 0x00:
    //     printf("red:%d",rec);
    //     break;
    //   case 0x01:
    //     printf("blue:%d",rec);
    //     break;
    //  }
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