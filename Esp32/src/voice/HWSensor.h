#ifndef HWSENSOR_H
#define HWSENSOR_H

#include <Arduino.h>

#define TTS_MODULE_I2C_ADDR    0x40

#define ASR_IIC_ADDR  0x34
#define ASR_RESULT_ADDR           0x64
//识别结果存放处，通过不断读取此地址的值判断是否识别到语音，不同的值对应不同的语音，
#define ASR_SPEAK_ADDR    0x6E

#define ASR_CMDMAND    0x00
#define ASR_ANNOUNCER  0xFF

class HWSensor {
  public:
    HWSensor();
    bool wireWriteByte(uint8_t addr, uint8_t val);
    bool wireWriteDataArray(uint8_t addr, uint8_t reg,uint8_t *val,unsigned int len);
    int wireReadDataArray(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len);

    /*语音合成模块*/
    bool ttsSpeak(unsigned char *sign,unsigned char *words);

    /*WonderEcho语音交互模块*/
    unsigned char asrGetResult(void);
    void asr_speak(uint8_t cmd , uint8_t id);
};
#endif
