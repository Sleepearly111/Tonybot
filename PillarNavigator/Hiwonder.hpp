#ifndef __HIWONDER_H
#define __HIWONDER_H

#include "stdint.h"
#include <Ticker.h>
#include "src/IMU/iic_sensor_task.h"

#define CHANNEL_DEFAULT 10

typedef enum {
    BUZZER_STAGE_START_NEW_CYCLE,
    BUZZER_STAGE_WATTING_OFF,
    BUZZER_STAGE_WATTING_PERIOD_END,
    BUZZER_STAGE_IDLE,
} BuzzerStageEnum;

class Buzzer_t{
    public:
        void init(uint8_t pin , uint8_t channel = CHANNEL_DEFAULT , uint16_t frequency = 1500);
        void on_off(uint8_t state);
        void blink(uint16_t frequency , uint16_t on_time , uint16_t off_time , uint16_t count);

    public:
        uint8_t buzzer_pin;
        uint8_t buzzer_channel;
        Ticker timer_buzzer;
        uint8_t new_flag;
        uint16_t freq;
        uint16_t ticks_on;
        uint16_t ticks_off;
        uint16_t repeat;
        BuzzerStageEnum stage;
        uint32_t ticks_count;
};

class IMU{
  public:
    void begin();
    void get_angle(float* roll , float* pitch);
};

#endif //__HIWONDER_H
