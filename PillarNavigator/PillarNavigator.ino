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
    SEARCH_PILLAR,  // 两段之间转头搜索第二根柱子
    DONE
};

static MainState main_state = GO_TO_PILLAR;

// 搜索第二根柱子
static uint8_t  search_phase = 0;
static unsigned long search_timer = 0;
static const uint8_t search_angles[] = {90, 45, 135};

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
    //pillarRoute_start(ROUTE_BLUE_RIGHT);
    pillarRoute_setRounds(6, 9);  // 第一段6轮，第二段9轮
    // pillarRoute_start(ROUTE_BLUE_LEFT);
     pillarRoute_start(ROUTE_RED_RIGHT);
    // pillarRoute_start(ROUTE_RED_LEFT);

    tracker_set_color(TRACK_COLOR_RED);
    Serial.println("start.");
}

void loop() {
    switch (main_state) {

        case GO_TO_PILLAR: {
            objectFollower_update(pillarRoute_color());
            if (get_width() > 220) {
                Controller.runActionGroup(0, 1);  // 先直立
                delay(800);
                circlePillar_init(pillarRoute_rounds());
                main_state = CIRCLE_PILLAR;
            }
            break;
        }

        case CIRCLE_PILLAR: {
            circlePillar_update(pillarRoute_color(), pillarRoute_direction());
            // 绕柱完成（内部满轮数前进自动DONE）
            if (circlePillar_isDone()) {
                if (pillarRoute_next()) {
                    // 两根柱子全部绕完
                    main_state = DONE;
                } else {
                    // 切到第二根柱子，先搜索目标
                    tracker_set_color(pillarRoute_color());
                    headTracker_resetToCenter();
                    search_phase = 0;
                    search_timer = millis();
                    main_state = SEARCH_PILLAR;
                }
            }
            break;
        }

        case SEARCH_PILLAR: {
            tracker_update();
            uint8_t w = get_width();

            if (w > 0) {
                Serial.print("[搜索] 找到目标 角度=");
                Serial.print(search_angles[search_phase]);
                Serial.print(" 宽度=");
                Serial.println(w);
                objectFollower_init();
                main_state = GO_TO_PILLAR;
                break;
            }

            // 等 500ms 让摄像头稳定
            if (millis() - search_timer < 500) break;

            // 切换到下一个角度
            search_phase = (search_phase + 1) % 3;
            uint8_t angle = search_angles[search_phase];
            Serial.print("[搜索] 转头到 ");
            Serial.println(angle);
            sonarServo.write(angle);
            search_timer = millis();
            break;
        }

        case DONE:
            break;
    }

    delay(50);
}
