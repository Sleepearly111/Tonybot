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
#include "PillarRoute.hpp"

enum MainState {
    GO_TO_PILLAR,
    CIRCLE_PILLAR,
    DONE
};

static MainState main_state = GO_TO_PILLAR;

LobotServoController Controller(Serial2);
HWSensor hwsensor;
Servo sonarServo;
HW_ESP32Cam hw_cam;

void setup() {
    Serial.begin(115200);
    fallDetector_init();
    Serial2.begin(9600, SERIAL_8N1, IO_BaseRX, IO_BaseTX);
    sonarServo.attach(IO_Servo);
    sonarServo.write(90);
    delay(200);
    Controller.runActionGroup(0, 1);
    delay(1500);
    hwsensor.ultrasoundColor(0, 0, 0, 0, 0, 0);
    delay(2000);

    // ========== 一句话选路线 ==========
    pillarRoute_start(ROUTE_BLUE_RIGHT);
    // pillarRoute_start(ROUTE_BLUE_LEFT);
    // pillarRoute_start(ROUTE_RED_RIGHT);
    // pillarRoute_start(ROUTE_RED_LEFT);

    tracker_set_color(TRACK_COLOR_RED);
    Serial.println("start.");
}

void loop() {
    switch (main_state) {

        case GO_TO_PILLAR: {
            objectFollower_update(pillarRoute_color());
            if (get_width() > 230) {
                circlePillar_init();
                main_state = CIRCLE_PILLAR;
            }
            break;
        }

        case CIRCLE_PILLAR: {
            circlePillar_update(pillarRoute_color(), pillarRoute_direction());
            // 绕柱完成（内部满4轮前进自动DONE）
            if (circlePillar_isDone()) {
                if (pillarRoute_next()) {
                    // 两根柱子全部绕完
                    main_state = DONE;
                } else {
                    // 切到第二根柱子
                    headTracker_resetToCenter();
                    objectFollower_init();
                    main_state = GO_TO_PILLAR;
                }
            }
            break;
        }

        case DONE:
            break;
    }

    delay(50);
}
