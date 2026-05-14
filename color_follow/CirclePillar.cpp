#include "CirclePillar.hpp"
#include "HeadTracker.hpp"
#include "LobotServoController.h"
#include "FallDetector.hpp"

// 调试开关 (注释可关闭串口输出)
#define CIRCLE_DEBUG

extern LobotServoController Controller;

// 动作组编号
#define SLIDE_LEFT   11   // 左侧滑
#define SLIDE_RIGHT  12   // 右侧滑
#define TURN_LEFT    23   // 左转
#define TURN_RIGHT   24   // 右转
#define FORWARD      25   // 前进
#define TURN_LEFT_S  3    // 小幅左转
#define TURN_RIGHT_S 4    // 小幅右转
#define STAND        0    // 直立
#define QUICK_STAND  19   // 快速立正
#define STAND_COOLDOWN 500 // 立正冷却

// 侧身目标角度
// CIRCLE_LEFT(左绕/逆时针)：柱子应在机器人右侧，头右转 ≤ 60
// CIRCLE_RIGHT(右绕/顺时针)：柱子应在机器人左侧，头左转 ≥ 120
#define SIDEWAY_ANGLE_LEFT   60
#define SIDEWAY_ANGLE_RIGHT  120

// 冷却
#define ADJUST_COOLDOWN    4000
#define STEP_DURATION      500
#define ANGLE_MIN_LIMIT    10    // 头右转极限
#define ANGLE_MAX_LIMIT    170   // 头左转极限

// 绕柱阶段头部角度保持区间
#define CIRCLE_L_ANGLE_MIN  8
#define CIRCLE_L_ANGLE_MAX  60
#define CIRCLE_R_ANGLE_MIN  120
#define CIRCLE_R_ANGLE_MAX  165

#define CIRCLING_WIDTH_MAX  180  // 绕柱阶段最大宽度

enum class CircleState {
    SIDEWAY_POSITION,
    CIRCLING,
    DONE
};

static CircleState circle_state = CircleState::SIDEWAY_POSITION;
static CircleState circle_prev = CircleState::SIDEWAY_POSITION;
static bool circle_first_entry = true;

static uint8_t  circle_direction = CIRCLE_LEFT;
static int      circle_width_min = 130;
static int      circle_width_max = 175;
static bool     circle_done = false;

// 侧身就位阶段
static unsigned long last_adjust_time = 0;
static bool     adjust_cooldown = false;

// 绕柱阶段
static unsigned long circle_step_timer = 0;
static bool     step_in_progress = false;
static bool     extreme_recovery = false;
static bool     big_turn_done = false;
static unsigned long last_forward_time = 0;
static int      slide_235_count = 0;
static int      total_forward_steps = 0;  // 总前进计数
static bool     stand_pending = false;
static unsigned long stand_timer = 0;
static bool     was_forward = false;       // 上一步是否是前进

// 宽度稳定检测（连续3次相等则稳定）
static uint8_t  _width_last = 0;
static int      _width_cnt = 0;
static unsigned long _width_wait_start = 0;

static bool isWidthStable(uint8_t &stable_width) {
    uint8_t cur = get_width();
    if (_width_wait_start == 0) _width_wait_start = millis();

    if (cur == _width_last) {
        _width_cnt++;
    } else {
        _width_last = cur;
        _width_cnt = 1;
    }
    if (_width_cnt >= 3) {
        stable_width = cur;
        _width_wait_start = 0;
        return true;
    }
    if (millis() - _width_wait_start >= 10000) {
        stable_width = cur;
        _width_wait_start = 0;
        _width_cnt = 3;
#ifdef CIRCLE_DEBUG
        Serial.print("[宽度] 超时强制取值：");
        Serial.println(cur);
#endif
        return true;
    }
    return false;
}

static void resetWidthCheck() {
    _width_last = get_width();
    _width_cnt = 1;
    _width_wait_start = 0;
}

#ifdef CIRCLE_DEBUG
static const char* circleStateNames[] = {
    "侧身就位",
    "绕柱中",
    "已完成"
};
#endif

void circlePillar_init() {
    circle_state = CircleState::SIDEWAY_POSITION;
    circle_prev = circle_state;
    circle_done = false;
    last_adjust_time = 0;
    adjust_cooldown = false;
    step_in_progress = false;
    resetStableCheck();
    resetWidthCheck();
}

int circlePillar_getSteps() {
    return total_forward_steps;
}

bool circlePillar_isDone() {
    return circle_done;
}

void circlePillar_update(uint8_t color_reg, uint8_t direction) {
    circle_direction = direction;

    static uint8_t last_color = 0xFF;
    if (color_reg != last_color) {
        last_color = color_reg;
        tracker_set_color(color_reg);
        resetStableCheck();
        resetWidthCheck();
    }

    // 跌倒起立检测，起立中暂停绕柱
    fallDetector_update();
    if (fallDetector_isRecovering()) return;

    tracker_update();
    uint8_t obj_width = get_width();

    int target_angle = (circle_direction == CIRCLE_LEFT)
                       ? SIDEWAY_ANGLE_LEFT
                       : SIDEWAY_ANGLE_RIGHT;

#ifdef CIRCLE_DEBUG
    if (circle_state != circle_prev) {
        Serial.print("[绕柱] 进入状态：");
        Serial.println(circleStateNames[(int)circle_state]);
        circle_prev = circle_state;
    }
#endif

    if (circle_done) return;

    switch (circle_state) {

        // ================= 侧身就位 =================
        case CircleState::SIDEWAY_POSITION: {
            if (circle_first_entry) {
                circle_first_entry = false;
                last_adjust_time = 0;
                adjust_cooldown = false;
                resetStableCheck();
                resetWidthCheck();
            }

            if (adjust_cooldown) {
                if (millis() - last_adjust_time >= ADJUST_COOLDOWN) {
                    adjust_cooldown = false;
                    resetStableCheck();
                    resetWidthCheck();
                }
                break;
            }

            // 目标丢失：连续1.5秒没识别到，小步回调找目标
            static int lost_count = 0;
            static unsigned long lost_start = 0;
            if (obj_width == 0) {
                if (lost_start == 0) lost_start = millis();
                if (millis() - lost_start > 1000) {
#ifdef CIRCLE_DEBUG
                    Serial.println("[侧身就位] 目标丢失，小步回调");
#endif
                    if (circle_direction == CIRCLE_LEFT) {
                        Controller.runActionGroup(TURN_RIGHT_S, 2);
                    } else {
                        Controller.runActionGroup(TURN_LEFT_S, 2);
                    }
                    last_adjust_time = millis();
                    adjust_cooldown = true;
                    lost_start = 0;
                }
                break;
            }
            lost_start = 0;

            uint8_t stable_angle;
            if (!isAngleStable(stable_angle)) break;

            bool angle_ok = false;
            if (circle_direction == CIRCLE_LEFT) {
                angle_ok = (stable_angle <= target_angle);
            } else {
                angle_ok = (stable_angle >= target_angle);
            }

            bool width_ok = (obj_width >= circle_width_min && obj_width <= circle_width_max);

            if (!angle_ok) {
                if (circle_direction == CIRCLE_LEFT) {
#ifdef CIRCLE_DEBUG
                    Serial.println("[侧身就位] 角度偏，左转");
#endif
                    Controller.runActionGroup(TURN_LEFT, 1);
                } else {
#ifdef CIRCLE_DEBUG
                    Serial.println("[侧身就位] 角度偏，右转");
#endif
                    Controller.runActionGroup(TURN_RIGHT, 1);
                }
                last_adjust_time = millis();
                adjust_cooldown = true;
                break;
            }

            if (!width_ok) {
                if (obj_width > circle_width_max) {
                    if (circle_direction == CIRCLE_LEFT) {
#ifdef CIRCLE_DEBUG
                        Serial.println("[侧身就位] 太近，左侧滑远离");
#endif
                        Controller.runActionGroup(SLIDE_LEFT, 1);
                    } else {
#ifdef CIRCLE_DEBUG
                        Serial.println("[侧身就位] 太近，右侧滑远离");
#endif
                        Controller.runActionGroup(SLIDE_RIGHT, 1);
                    }
                } else {
                    if (circle_direction == CIRCLE_LEFT) {
#ifdef CIRCLE_DEBUG
                        Serial.println("[侧身就位] 太远，右侧滑靠近");
#endif
                        Controller.runActionGroup(SLIDE_RIGHT, 1);
                    } else {
#ifdef CIRCLE_DEBUG
                        Serial.println("[侧身就位] 太远，左侧滑靠近");
#endif
                        Controller.runActionGroup(SLIDE_LEFT, 1);
                    }
                }
                last_adjust_time = millis();
                adjust_cooldown = true;
                break;
            }

#ifdef CIRCLE_DEBUG
            Serial.print("[侧身就位] 就位完成 角度=");
            Serial.print(stable_angle);
            Serial.print(" 宽度=");
            Serial.println(obj_width);
#endif
            resetStableCheck();
            resetWidthCheck();
            last_forward_time = millis();
            big_turn_done = false;
            circle_state = CircleState::CIRCLING;
            break;
        }

        // ================= 绕柱行走 =================
        case CircleState::CIRCLING: {
            uint8_t stable_angle;
            if (!isAngleStable(stable_angle)) break;

            uint8_t stable_width;
            if (!isWidthStable(stable_width)) break;

            if (!step_in_progress) {
                // 立正冷却：等机器人站稳再判断
                if (stand_pending) {
                    if (millis() - stand_timer >= STAND_COOLDOWN) {
                        stand_pending = false;
                        resetStableCheck();
                        resetWidthCheck();
                    }
                    break;
                }

                // 6秒超时保护（极端角度或丢失时不强制前进）
                if (millis() - last_forward_time > 6000
                    && stable_angle > ANGLE_MIN_LIMIT
                    && stable_angle < ANGLE_MAX_LIMIT
                    && stable_width > 0) {
#ifdef CIRCLE_DEBUG
                    Serial.print("[绕柱] 调整超时6s，强制前进 总步数=");
                    Serial.println(total_forward_steps + 1);
#endif
                    last_forward_time = millis();
                    was_forward = true;
                    total_forward_steps++;
                    Controller.runActionGroup(FORWARD, 2);
                    circle_step_timer = millis();
                    step_in_progress = true;
                    slide_235_count = 0;
                    big_turn_done = false;
                    break;
                }

                // 紧急避让：width>235 侧滑，最多连续2次
                if (stable_width > 235) {
                    slide_235_count++;
                    if (slide_235_count > 2) {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 侧滑已达2次，强制前进 总步数=");
                        Serial.println(total_forward_steps + 1);
#endif
                        slide_235_count = 0;
                        was_forward = true;
                        total_forward_steps++;
                        Controller.runActionGroup(FORWARD, 2);
                    } else {
                        if (circle_direction == CIRCLE_LEFT) {
#ifdef CIRCLE_DEBUG
                            Serial.print("[绕柱] 紧急太近(");
                            Serial.print(stable_width);
                            Serial.print(") 侧滑第");
                            Serial.print(slide_235_count);
                            Serial.println("次");
#endif
                            Controller.runActionGroup(SLIDE_LEFT, 1);
                        } else {
#ifdef CIRCLE_DEBUG
                            Serial.print("[绕柱] 紧急太近(");
                            Serial.print(stable_width);
                            Serial.print(") 侧滑第");
                            Serial.print(slide_235_count);
                            Serial.println("次");
#endif
                            Controller.runActionGroup(SLIDE_RIGHT, 1);
                        }
                    }
                    circle_step_timer = millis();
                    step_in_progress = true;
                    break;
                } else {
                    slide_235_count = 0;
                }

                // 极端角度 + 目标丢失 → 根据最后色块坐标搜索
                if ((stable_angle <= ANGLE_MIN_LIMIT || stable_angle >= ANGLE_MAX_LIMIT)
                    && stable_width == 0) {
                    uint8_t lx = get_last_x();
                    bool turn_left = (lx < 120);  // 色块在左侧→左转，右侧→右转
                    if (!big_turn_done) {
                        big_turn_done = true;
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 极端丢失，最后x=");
                        Serial.print(lx);
                        Serial.println(turn_left ? " 大步左转搜索" : " 大步右转搜索");
#endif
                        Controller.runActionGroup(turn_left ? TURN_LEFT : TURN_RIGHT, 1);
                    } else {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 极端丢失，最后x=");
                        Serial.print(lx);
                        Serial.println(turn_left ? " 小步左转搜索" : " 小步右转搜索");
#endif
                        Controller.runActionGroup(turn_left ? TURN_LEFT_S : TURN_RIGHT_S, 2);
                    }
                    circle_step_timer = millis();
                    step_in_progress = true;
                    extreme_recovery = true;
                    break;
                }

                // 角度极限保护（大转仅一次）
                if (stable_angle <= ANGLE_MIN_LIMIT) {
                    if (!big_turn_done) {
                        big_turn_done = true;
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 头右极限(");
                        Serial.print(stable_angle);
                        Serial.println(")，大步右转");
#endif
                        Controller.runActionGroup(TURN_RIGHT, 1);
                    } else {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 头右极限(");
                        Serial.print(stable_angle);
                        Serial.println(")，小步右转");
#endif
                        Controller.runActionGroup(TURN_RIGHT_S, 2);
                    }
                    circle_step_timer = millis();
                    step_in_progress = true;
                    extreme_recovery = true;
                    break;
                }
                if (stable_angle >= ANGLE_MAX_LIMIT) {
                    if (!big_turn_done) {
                        big_turn_done = true;
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 头左极限(");
                        Serial.print(stable_angle);
                        Serial.println(")，大步左转");
#endif
                        Controller.runActionGroup(TURN_LEFT, 1);
                    } else {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 头左极限(");
                        Serial.print(stable_angle);
                        Serial.println(")，小步左转");
#endif
                        Controller.runActionGroup(TURN_LEFT_S, 1);
                    }
                    circle_step_timer = millis();
                    step_in_progress = true;
                    extreme_recovery = true;
                    break;
                }

                big_turn_done = false;

                // 距离不对 → 小幅转向
                if (stable_width > CIRCLING_WIDTH_MAX) {
                    if (circle_direction == CIRCLE_LEFT) {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 太近(");
                        Serial.print(stable_width);
                        Serial.println(")，小幅左转远离");
#endif
                        Controller.runActionGroup(TURN_LEFT_S, 1);
                    } else {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 太近(");
                        Serial.print(stable_width);
                        Serial.println(")，小幅右转远离");
#endif
                        Controller.runActionGroup(TURN_RIGHT_S, 1);
                    }
                    circle_step_timer = millis();
                    step_in_progress = true;
                    break;
                }

                if (stable_width < circle_width_min) {
                    if (circle_direction == CIRCLE_LEFT) {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 太远(");
                        Serial.print(stable_width);
                        Serial.println(")，小幅右转靠近");
#endif
                        Controller.runActionGroup(TURN_RIGHT_S, 1);
                    } else {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 太远(");
                        Serial.print(stable_width);
                        Serial.println(")，小幅左转靠近");
#endif
                        Controller.runActionGroup(TURN_LEFT_S, 1);
                    }
                    circle_step_timer = millis();
                    step_in_progress = true;
                    break;
                }

                // 角度偏离 → 小幅转向
                {
                    int angle_min, angle_max;
                    uint8_t turn_low, turn_high;
                    if (circle_direction == CIRCLE_LEFT) {
                        angle_min = CIRCLE_L_ANGLE_MIN;
                        angle_max = CIRCLE_L_ANGLE_MAX;
                        turn_low  = TURN_RIGHT_S;
                        turn_high = TURN_LEFT_S;
                    } else {
                        angle_min = CIRCLE_R_ANGLE_MIN;
                        angle_max = CIRCLE_R_ANGLE_MAX;
                        turn_low  = TURN_RIGHT_S;
                        turn_high = TURN_LEFT_S;
                    }
                    if (stable_angle < angle_min) {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 角度偏低(");
                        Serial.print(stable_angle);
                        Serial.println(")，小幅矫正");
#endif
                        Controller.runActionGroup(turn_low, 1);
                        circle_step_timer = millis();
                        step_in_progress = true;
                        break;
                    }
                    if (stable_angle > angle_max) {
#ifdef CIRCLE_DEBUG
                        Serial.print("[绕柱] 角度偏高(");
                        Serial.print(stable_angle);
                        Serial.println(")，小幅矫正");
#endif
                        Controller.runActionGroup(turn_high, 1);
                        circle_step_timer = millis();
                        step_in_progress = true;
                        break;
                    }
                }

                // 全OK，前进
#ifdef CIRCLE_DEBUG
                Serial.print("[绕柱] 前进 宽度=");
                Serial.print(stable_width);
                Serial.print(" 总步数=");
                Serial.println(total_forward_steps);
#endif
                last_forward_time = millis();
                big_turn_done = false;
                slide_235_count = 0;
                was_forward = true;
                Controller.runActionGroup(FORWARD, 2);
                circle_step_timer = millis();
                step_in_progress = true;
            } else {
                unsigned long wait_time = extreme_recovery ? 3000 : STEP_DURATION;
                if (millis() - circle_step_timer >= wait_time) {
                    step_in_progress = false;
                    if (extreme_recovery) {
                        extreme_recovery = false;
#ifdef CIRCLE_DEBUG
                        Serial.println("[绕柱] 极端恢复完成");
#endif
                    }
                    if (was_forward) {
                        was_forward = false;
                        Controller.runActionGroup(QUICK_STAND, 1);
                        stand_pending = true;
                        stand_timer = millis();
                    } else {
                        resetStableCheck();
                        resetWidthCheck();
                    }
                }
            }
            break;
        }

        // ================= 完成 =================
        case CircleState::DONE: {
#ifdef CIRCLE_DEBUG
            Serial.println("[绕柱] 头部回中，直立");
#endif
            headTracker_resetToCenter();
            Controller.runActionGroup(STAND, 1);
            circle_done = true;
            break;
        }
    }
}
