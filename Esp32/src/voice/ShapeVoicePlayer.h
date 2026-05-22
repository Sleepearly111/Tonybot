#ifndef SHAPE_VOICE_PLAYER_H
#define SHAPE_VOICE_PLAYER_H

// 1. 引入硬件核心配置（完整引脚定义）
#include "../base_config.h"
// 2. 引入I2C通信核心库（必须，ESP32 I2C依赖）
#include <Wire.h>
// 3. 引入TTS语音模块驱动（同目录引用，确保完整）
#include "HWSensor.h"
// 4. 引入Arduino核心库（delay/数据类型等基础依赖）
#include <Arduino.h>

/**
 * 形状语音播报器类（完整适配base_config.h所有硬件定义）
 * 功能：播报球体/正方体/圆柱体（各3遍），仅使用I2C引脚(22/23)，不占用其他硬件资源
 */
class ShapeVoicePlayer {
private:
    HWSensor hwSensor;  // TTS模块实例（绑定base_config的I2C引脚）
    
    /**
     * 完整的重复播报辅助函数（无冗余/无遗漏）
     * @param ttsSign TTS控制指令（音量/语速等，v[10]=音量10）
     * @param words 播报文字（GB2312编码，适配TTS模块）
     * @param repeatTimes 重复次数（默认3次，与需求一致）
     */
    void speakRepeatedly(const char* ttsSign, const char* words, int repeatTimes);

public:
    /** 构造函数：完整初始化I2C和TTS模块 */
    ShapeVoicePlayer();
    
    /** 球体播报（完整实现，无功能缺失） */
    void playSphere();
    
    /** 正方体播报（完整实现，无功能缺失） */
    void playCube();
    
    /** 圆柱体播报（完整实现，无功能缺失） */
    void playCylinder();
};

#endif // SHAPE_VOICE_PLAYER_H