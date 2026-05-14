/**************************************************************
 * company：深圳市幻尔科技有限公司
 * date&&author：20241128&&CuZn
 * description：S形双柱绕行
**************************************************************/
#include "base_config.h"
#include "HardwareSerial.h"
#include "LobotServoController.h"
#include "Arduino.h"
#include "Servo.h"
#include "HWSensor.h"
#include "Hiwonder.hpp"
#include "hw_esp32cam_ctl.h"
#include "HeadTracker.hpp"
#include "ObjectFollower.hpp"
#include "CirclePillar.hpp"
#include "FallDetector.hpp"

#define TURN_LEFT  23
#define TURN_RIGHT 24

enum MainState {
    GO_TO_BLUE,
    CIRCLE_BLUE,
    GO_TO_RED,
    CIRCLE_RED
};

static MainState main_state = GO_TO_BLUE;

LobotServoController Controller(Serial2);
HWSensor hwsensor;
Servo sonarServo;
HW_ESP32Cam hw_cam;

void setup() {
    Serial.begin(115200);
    fallDetector_init();  // IMU初始化+6s预热
    Serial2.begin(9600, SERIAL_8N1, IO_BaseRX, IO_BaseTX);
    sonarServo.attach(IO_Servo);
    sonarServo.write(90);
    delay(200);
    Controller.runActionGroup(0, 1);
    delay(1500);
    hwsensor.ultrasoundColor(0, 0, 0, 0, 0, 0);
    delay(2000);
    tracker_set_color(TRACK_COLOR_RED);
    Serial.println("start.");
}

void switchTo(MainState s) {
    main_state = s;
#ifdef CIRCLE_DEBUG
    const char* names[] = {"走向蓝柱", "绕蓝柱", "走向红柱", "绕红柱"};
    Serial.print("[主状态] → ");
    Serial.println(names[s]);
#endif
}

void loop() {
    switch (main_state) {

        case GO_TO_BLUE: {
            objectFollower_update(TRACK_COLOR_BLUE);
            // 到达蓝色柱子
            if (get_width() > 230) {
                switchTo(CIRCLE_BLUE);
                circlePillar_init();
            }
            break;
        }

        case CIRCLE_BLUE: {
            circlePillar_update(TRACK_COLOR_BLUE, CIRCLE_RIGHT);
            // 绕蓝柱满6步，切换红色走向红柱
            if (circlePillar_getSteps() >= 6) {
                circlePillar_init();
                headTracker_resetToCenter();
                tracker_set_color(TRACK_COLOR_RED);
                objectFollower_init();
                switchTo(GO_TO_RED);
            }
            break;
        }

        case GO_TO_RED: {
            objectFollower_update(TRACK_COLOR_RED);
            // 到达红色柱子
            if (get_width() > 230) {
                switchTo(CIRCLE_RED);
                circlePillar_init();
            }
            break;
        }

        case CIRCLE_RED: {
            circlePillar_update(TRACK_COLOR_RED, CIRCLE_LEFT);
            break;
        }
    }

    delay(50);
}
