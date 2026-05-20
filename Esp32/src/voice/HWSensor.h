#ifndef HWSENSOR_H
#define HWSENSOR_H

#include <Arduino.h>

#define ULTRASOUND_I2C_ADDR 0x77 

//寄存器
#define DISDENCE_L    0//距离低8位，单位mm
#define DISDENCE_H    1

#ifndef RGB_BRIGHTNESS
#define RGB_BRIGHTNESS  50//0-255
#endif

#define RGB_WORK_MODE 2//RGB灯模式，0：用户自定义模式   1：呼吸灯模式  默认0

#define RGB1_R      3//1号探头的R值，0~255，默认0
#define RGB1_G      4//默认0
#define RGB1_B      5//默认255

#define RGB2_R      6//2号探头的R值，0~255，默认0
#define RGB2_G      7//默认0
#define RGB2_B      8//默认255

#define RGB1_R_BREATHING_CYCLE      9 //呼吸灯模式时，1号探头的R的呼吸周期，单位100ms 默认0，
                                      //如果设置周期3000ms，则此值为30
#define RGB1_G_BREATHING_CYCLE      10
#define RGB1_B_BREATHING_CYCLE      11

#define RGB2_R_BREATHING_CYCLE      12//2号探头
#define RGB2_G_BREATHING_CYCLE      13
#define RGB2_B_BREATHING_CYCLE      14

#define RGB_WORK_SIMPLE_MODE    0
#define RGB_WORK_BREATHING_MODE   1

#define TTS_MODULE_I2C_ADDR    0x40

#define ASR_IIC_ADDR  0x34
#define ASR_RESULT_ADDR           0x64
//识别结果存放处，通过不断读取此地址的值判断是否识别到语音，不同的值对应不同的语音，
#define ASR_SPEAK_ADDR    0x6E

#define ASR_CMDMAND    0x00
#define ASR_ANNOUNCER  0xFF

#define IIC_FAN_ADDR      0x39

class HWSensor {
  public:
    HWSensor();
    bool wireWriteByte(uint8_t addr, uint8_t val);
    bool wireWriteDataArray(uint8_t addr, uint8_t reg,uint8_t *val,unsigned int len);
    int wireReadDataArray(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len);

    /*发光超声波*/
    void ultrasoundBreathing(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2);
    void ultrasoundColor(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2);
    uint16_t ultrasoundGetDistance();

    /*语音合成模块*/
    bool ttsSpeak(unsigned char *sign,unsigned char *words);

    /*温湿度传感器*/
    float getTemperature(void);// 获取温度值
    float getHumidity(void);// 获取湿度值

    /*WonderEcho语音交互模块*/
    unsigned char asrGetResult(void);
    void asr_speak(uint8_t cmd , uint8_t id);

    /*风扇（IIC接口）*/
    void fanSpeed(int8_t speed);
};
#endif
