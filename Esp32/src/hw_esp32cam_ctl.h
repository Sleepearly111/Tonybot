/* 
 * 2024/02/21 hiwonder CuZn
 * Arduino与ESP32Cam的IIC通讯类
 * 注意：每个不同的功能，ESP32Cam也要下载对应功能的ESP32Cam程序
 */

#ifndef __HW_ESP32CAM_CTL_H_
#define __HW_ESP32CAM_CTL_H_

#include <Arduino.h>
#include <Wire.h>

#define ESP32CAM_ADDR 0x52

class HW_ESP32Cam{
  public:
    //初始化IIC
    void begin(void);
    //人脸识别获取函数
    bool faceDetect(void);
    //颜色识别获取函数
    int colorDetect(void);
    // 颜色位置获取函数
    bool color_position(uint8_t *color_info);
    //补光灯亮度设置函数 ( 0 ~ 255 )
    void set_led(uint8_t lightness);
    // AI 识别结果读取（S3 视觉协处理器）
    uint8_t readAILabel();   // 读取标签结果 (寄存器 0x10) — 0x11=球体 0x12=正方体 0x13=圆柱体
    uint8_t readAIShape();   // 读取形状结果 (寄存器 0x20) — 1=球体 2=正方体 3=圆柱体
};

#endif //__ESP32CAM_CTL_H_

