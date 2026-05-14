#include "FallDetector.hpp"
#include "Hiwonder.hpp"
#include "LobotServoController.h"
#include "base_config.h"

extern LobotServoController Controller;

static IMU imu;
static Buzzer_t buzzer_obj;

static float radianX, radianY;

enum class FallState {
    NORMAL,
    FALLEN_FRONT,
    FALLEN_BACK,
    RECOVERING
};

static FallState fall_state = FallState::NORMAL;
static uint32_t fall_timer = 0;
static uint8_t count_front = 0;
static uint8_t count_back = 0;

void fallDetector_init() {
    imu.begin();
    buzzer_obj.init(IO_BUZZER);
    delay(6000); // IMU 预热
    buzzer_obj.blink(1500, 100, 100, 2);
}

void fallDetector_update() {
    if (fall_timer > millis()) return;

    switch (fall_state) {
        case FallState::NORMAL:
            imu.get_angle(&radianX, &radianY);
            // 前倒判定（几乎完全倒下）
            if (radianX < 20 && radianX > -20) {
                count_front++;
                count_back = 0;
                fall_timer = millis() + 50;
                if (count_front > 100) {
                    count_front = 0;
                    fall_state = FallState::FALLEN_FRONT;
                    buzzer_obj.blink(1500, 100, 100, 1);
                    fall_timer = millis() + 1000;
                }
            }
            // 后倒判定
            else if (radianX > 160 || radianX < -170) {
                count_back++;
                count_front = 0;
                fall_timer = millis() + 50;
                if (count_back > 100) {
                    count_back = 0;
                    fall_state = FallState::FALLEN_BACK;
                    buzzer_obj.blink(1500, 100, 100, 1);
                    fall_timer = millis() + 1000;
                }
            }
            // 正常
            else {
                count_front = 0;
                count_back = 0;
            }
            break;

        case FallState::FALLEN_FRONT:
            Controller.runActionGroup(102, 1);
            fall_timer = millis() + 7000;
            fall_state = FallState::RECOVERING;
            break;

        case FallState::FALLEN_BACK:
            Controller.runActionGroup(101, 1);
            fall_timer = millis() + 7000;
            fall_state = FallState::RECOVERING;
            break;

        case FallState::RECOVERING:
            fall_state = FallState::NORMAL;
            break;
    }
}

bool fallDetector_isRecovering() {
    return (fall_state != FallState::NORMAL);
}
