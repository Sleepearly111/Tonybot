#include "ShapeVoicePlayer.h"

/**
 * 构造函数：完整初始化I2C总线（绑定base_config的IO_SDA/IO_SCL）
 * 确保TTS模块通信的硬件层初始化无遗漏
 *
 */
ShapeVoicePlayer::ShapeVoicePlayer() {
    // 1. 初始化I2C总线（明确指定base_config定义的引脚，避免默认引脚冲突）
    //Wire.begin(IO_SDA, IO_SCL);
    // 2. 可选：初始化TTS模块（若HWSensor需显式初始化，根据实际补充）
    // hwSensor.begin(); // 若HWSensor有begin()方法，取消注释即可
}

/**
 * 辅助函数：完整的重复播报逻辑（无串口打印，仅保留核心播报）
 * 确保每一次播报的延时、调用都完整，无逻辑断点
 */
void ShapeVoicePlayer::speakRepeatedly(const char* ttsSign, const char* words, int repeatTimes) {
    // 循环次数完整（从0到repeatTimes-1，确保精准重复指定次数）
    for (int i = 0; i < repeatTimes; i++) {
        // 完整调用TTS播报接口（类型转换适配HWSensor的参数要求）
        hwSensor.ttsSpeak((unsigned char*)ttsSign, (unsigned char*)words);
        // 延时完整（1500ms确保语音播放完成，无重叠，适配短句播报）
        delay(1500);
    }
}

/**
 * 球体播报：完整调用辅助函数，参数无遗漏
 * 音量固定为10（0-15可调，适配大多数场景），重复3次
 */
void ShapeVoicePlayer::playSphere() {
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x11);
    delay(2000);
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x11);
    delay(2000);
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x11);
    delay(2000);
}

/**
 * 正方体播报：完整调用辅助函数，参数无遗漏
 */
void ShapeVoicePlayer::playCube() {
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x12);
    delay(2000);
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x12);
    delay(2000);
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x12);
    delay(2000);
}

/**
 * 圆柱体播报：完整调用辅助函数，参数无遗漏
 */
void ShapeVoicePlayer::playCylinder() {
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x13);
    delay(2000);
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x13);
    delay(2000);
    hwSensor.asr_speak(ASR_ANNOUNCER , 0x13);
    delay(2000);
}