#include "ObjectFollower.hpp"
#include "HeadTracker.hpp"
#include "LobotServoController.h"

// 调试开关 (注释可关闭串口输出)
#define OBJECT_FOLLOWER_DEBUG

extern LobotServoController Controller;

// 动作组编号（根据你的实际硬件修改）
#define TURN_LEFT_GROUP  23
#define TURN_RIGHT_GROUP 24
#define FORWARD_GROUP    25

// 每次对准后前进的步数
#define FORWARD_STEPS    4
// 每步预估耗时 (毫秒)
#define STEP_DURATION    500
// 走完全部步数后的稳定停顿 (毫秒)
#define POST_FORWARD_PAUSE 300
// 转向后的冷却等待时间 (毫秒) – 给头部足够时间转动并锁定物体
#define TURN_COOLDOWN_MS 3500

// 右侧额外允许的偏差 (因为摄像头偏右，可根据测试调整)
#define RIGHT_BIAS 15

// 物体宽度阈值：稳定大于此值视为已到达物体面前
#define ARRIVED_WIDTH        220
#define ARRIVED_STABLE_COUNT 3

enum class FollowState {
    WAIT_STABLE,      // 等待头部稳定
    DECIDE,           // 根据稳定角度决策
    TURNING,          // 转向中 (一次转一下，等稳定再判断)
    FORWARD_EXEC,     // 执行前进动作
    FORWARD_CHECK,    // 前进每步后等待稳定并检查偏离
    FORWARD_DONE,     // 全部步数完成，短暂停顿后重新观察
    ARRIVED           // 已到达物体面前，头部回中并直立
};

static FollowState state = FollowState::WAIT_STABLE;
static FollowState prev_state = FollowState::WAIT_STABLE;
static uint8_t target_angle = 90;
static int stable_deadband = 20;
static int forward_low  = 70;
static int forward_high = 110;
static int rough_deadband = 30;
static int steps_remaining = 0;
static unsigned long step_timer = 0;
static unsigned long stable_block_time = 0;   // 用于 WAIT_STABLE 的最小等待时间戳

// TURNING 状态专用变量
static unsigned long turn_state_entry = 0;
static unsigned long last_turn_cmd_time = 0;
static bool turn_cooldown = false;

// 记录 TURNING 序列中上一次转向方向 (1=左转，-1=右转，0=无)
static int turning_last_dir = 0;

// 宽度稳定检测：连续大于 ARRIVED_WIDTH 的计数
static int width_stable_count = 0;
// 标记已到达动作是否已执行（只执行一次）
static bool arrived_executed = false;
// 至少执行过一次前进后才允许判定到达
static bool has_approached = false;

#ifdef OBJECT_FOLLOWER_DEBUG
static const char* stateNames[] = {
    "等待稳定",
    "决策",
    "转向中",
    "前进执行",
    "前进检查",
    "前进完成",
    "已到达"
};
#endif

void objectFollower_init() {
    state = FollowState::WAIT_STABLE;
    prev_state = state;
    target_angle = 90;
    steps_remaining = 0;
    turn_state_entry = 0;
    last_turn_cmd_time = 0;
    turn_cooldown = false;
    turning_last_dir = 0;
    width_stable_count = 0;
    arrived_executed = false;
    has_approached = false;
    stable_block_time = millis();  // 首次进入等300ms让摄像头跟踪
}

void objectFollower_update(uint8_t color_reg, int deadband) {
    // 仅在颜色变化时切换探测颜色
    static uint8_t last_obj_color = 0xFF;
    if (color_reg != last_obj_color) {
        last_obj_color = color_reg;
        tracker_set_color(color_reg);
    }

    // 更新死区参数，直行区间保持静态默认 [70, 110]
    stable_deadband = deadband;
    rough_deadband = deadband * 3;
    int rough_low  = 90 - rough_deadband;
    int rough_high = 90 + rough_deadband;

    // 每帧必须更新头部跟踪
    tracker_update();
    uint8_t current_angle = get_head_angle();

#ifdef OBJECT_FOLLOWER_DEBUG
    if (state != prev_state) {
        Serial.print("[状态] 进入：");
        Serial.println(stateNames[(int)state]);
        prev_state = state;
    }
#endif

    // 宽度稳定检测：当物体宽度持续大于阈值，判定已到达
    uint8_t cur_width = get_width();
    if (cur_width > ARRIVED_WIDTH && state != FollowState::ARRIVED && has_approached) {
        width_stable_count++;
        if (width_stable_count >= ARRIVED_STABLE_COUNT) {
#ifdef OBJECT_FOLLOWER_DEBUG
            Serial.print("[宽度检测] 宽度稳定 > ");
            Serial.print(ARRIVED_WIDTH);
            Serial.print(" (");
            Serial.print(cur_width);
            Serial.println(")，已到达物体面前");
#endif
            state = FollowState::ARRIVED;
            width_stable_count = 0;
        }
    } else {
        width_stable_count = 0;
    }

    switch (state) {
        // ================= 等待稳定 =================
        case FollowState::WAIT_STABLE: {
    // 进入该状态后必须等待至少 200ms，给摄像头足够时间重新捕捉物体并调整头部
    if (millis() - stable_block_time < 300) {
        // 阻塞期间仍然每帧运行 tracker_update（在 switch 之前已调用），
        // 所以头部会随物体移动，角度会变化，不会一直停在 90°
        break;
    }

    // 等待时间已过，开始正常的角度稳定判断
    if (isAngleStable(target_angle)) {
#ifdef OBJECT_FOLLOWER_DEBUG
        Serial.print("[等待稳定] 头部已稳定，角度：");
        Serial.println(target_angle);
#endif
        state = FollowState::DECIDE;
    }
    break;
}

        // ================= 决策 =================
        case FollowState::DECIDE: {
#ifdef OBJECT_FOLLOWER_DEBUG
            Serial.print("[决策] 目标角度：");
            Serial.print(target_angle);
            Serial.print("，允许区间：[");
            Serial.print(forward_low);
            Serial.print(", ");
            Serial.print(forward_high);
            Serial.println("]");
#endif
            // 是否在允许前进的区间内？
            if (target_angle >= forward_low && target_angle <= forward_high) {
#ifdef OBJECT_FOLLOWER_DEBUG
                Serial.println("[决策] 在允许区间内，开始前进");
#endif
                steps_remaining = FORWARD_STEPS;
                has_approached = true;
                Controller.runActionGroup(FORWARD_GROUP, 1);
                step_timer = millis();
                state = FollowState::FORWARD_EXEC;
            }
            // 目标偏左 → 身体右转
            else if (target_angle < forward_low) {
#ifdef OBJECT_FOLLOWER_DEBUG
                Serial.println("[决策] 目标偏左，执行右转");
#endif
                Controller.runActionGroup(TURN_RIGHT_GROUP, 1);
                turn_state_entry = millis();
                last_turn_cmd_time = millis();
                turn_cooldown = true;
                turning_last_dir = -1;   // 记录右转
                state = FollowState::TURNING;
            }
            // 目标偏右 → 身体左转
            else {
#ifdef OBJECT_FOLLOWER_DEBUG
                Serial.println("[决策] 目标偏右，执行左转");
#endif
                Controller.runActionGroup(TURN_LEFT_GROUP, 1);
                turn_state_entry = millis();
                last_turn_cmd_time = millis();
                turn_cooldown = true;
                turning_last_dir = 1;    // 记录左转
                state = FollowState::TURNING;
            }
            break;
        }

        // ================= 转向中 =================
        case FollowState::TURNING: {
            // 超时保护 (10 秒)
            if (millis() - turn_state_entry > 10000) {
#ifdef OBJECT_FOLLOWER_DEBUG
                Serial.println("[转向中] 超时，强制退出并回正");
#endif
                headTracker_resetToCenter();
                resetStableCheck();
                stable_block_time = millis();
                turning_last_dir = 0;
                state = FollowState::WAIT_STABLE;
                break;
            }

            // 极值保护：角度到达 0 或 180 说明识别错误，立刻回中
            if (current_angle == 0 || current_angle == 180) {
#ifdef OBJECT_FOLLOWER_DEBUG
                Serial.print("[转向中] 角度异常 (");
                Serial.print(current_angle);
                Serial.println(")，识别错误，回中退出");
#endif
                headTracker_resetToCenter();
                resetStableCheck();
                stable_block_time = millis();
                turning_last_dir = 0;
                state = FollowState::WAIT_STABLE;
                break;
            }

            // 转向后冷却期：等待头部转动并锁定物体
            if (turn_cooldown) {
                if (millis() - last_turn_cmd_time >= TURN_COOLDOWN_MS) {
                    turn_cooldown = false;
                    resetStableCheck();
                }
                break;  // 冷却期间不做任何判断
            }

            // 等待头部稳定
            uint8_t stable_angle;
            if (isAngleStable(stable_angle)) {
                // 是否已在允许的区间内？
                if (stable_angle >= forward_low && stable_angle <= forward_high) {
#ifdef OBJECT_FOLLOWER_DEBUG
                    Serial.print("[转向中] 稳定后已进入允许区间，角度：");
                    Serial.println(stable_angle);
#endif
                    headTracker_resetToCenter();
                    resetStableCheck();
                    stable_block_time = millis();
                    turning_last_dir = 0;
                    state = FollowState::WAIT_STABLE;
                } else {
                    // 检查是否连续两次同方向转向（判错）
                    int current_dir = (stable_angle < forward_low) ? -1 : 1;
                    if (turning_last_dir != 0 && current_dir == turning_last_dir) {
#ifdef OBJECT_FOLLOWER_DEBUG
                        Serial.println("[转向中] 连续两次转向方向相同，判断错误，回正重找物体");
#endif
                        headTracker_resetToCenter();
                        resetStableCheck();
                        stable_block_time = millis();
                        turning_last_dir = 0;
                        state = FollowState::WAIT_STABLE;
                        break;
                    }

                    // 仍在区间外，继续转向
                    if (stable_angle < forward_low) {
#ifdef OBJECT_FOLLOWER_DEBUG
                        Serial.println("[转向中] 稳定后仍偏左，再右转一次");
#endif
                        Controller.runActionGroup(TURN_RIGHT_GROUP, 1);
                        turning_last_dir = -1;
                    } else {
#ifdef OBJECT_FOLLOWER_DEBUG
                        Serial.println("[转向中] 稳定后仍偏右，再左转一次");
#endif
                        Controller.runActionGroup(TURN_LEFT_GROUP, 1);
                        turning_last_dir = 1;
                    }
                    last_turn_cmd_time = millis();
                    turn_cooldown = true;

                    // 目标丢失检查：连续多次偏离且角度不变
                    static int consecutive_fail = 0;
                    static int last_stable = -1;
                    if (abs(stable_angle - last_stable) <= 2) {
                        consecutive_fail++;
                    } else {
                        consecutive_fail = 0;
                    }
                    last_stable = stable_angle;
                    if (consecutive_fail >= 3) {
#ifdef OBJECT_FOLLOWER_DEBUG
                        Serial.println("[转向中] 连续多次偏离且角度不变，目标可能丢失，回正退出");
#endif
                        headTracker_resetToCenter();
                        resetStableCheck();
                        stable_block_time = millis();
                        consecutive_fail = 0;
                        turning_last_dir = 0;
                        state = FollowState::WAIT_STABLE;
                    }
                }
            }
            break;
        }

        // ================= 前进执行 =================
        case FollowState::FORWARD_EXEC: {
            // 等待当前一步的动作完成
            if (millis() - step_timer >= STEP_DURATION) {
                steps_remaining--;
#ifdef OBJECT_FOLLOWER_DEBUG
                Serial.print("[前进执行] 完成一步，剩余步数：");
                Serial.println(steps_remaining);
#endif
                if (steps_remaining > 0) {
                    // 还有剩余的步数，先进入检查状态等待稳定
                    resetStableCheck();
                    state = FollowState::FORWARD_CHECK;
                } else {
                    // 全部步数完成，进入结束停顿状态
                    step_timer = millis();
                    state = FollowState::FORWARD_DONE;
                }
            }
            break;
        }

        // ================= 前进检查（每步后等稳定，防偏离） =================
        case FollowState::FORWARD_CHECK: {
            // 每步后强制等300ms，让身体站稳、摄像头重新跟踪
            static unsigned long fwd_check_entry = 0;
            static bool fwd_need_wait = true;
            if (fwd_need_wait) {
                fwd_need_wait = false;
                fwd_check_entry = millis();
            }
            if (millis() - fwd_check_entry < 300) break;

            uint8_t stable_angle;
            if (isAngleStable(stable_angle)) {
                fwd_need_wait = true;  // 下次进入重新计时
                // 检查是否仍在允许的区间内
                if (stable_angle >= forward_low && stable_angle <= forward_high) {
#ifdef OBJECT_FOLLOWER_DEBUG
                    Serial.println("[前进检查] 稳定且未偏离，继续下一步");
#endif
                    Controller.runActionGroup(FORWARD_GROUP, 1);
                    step_timer = millis();
                    state = FollowState::FORWARD_EXEC;
                } else {
                    // 偏离了，直接用当前稳定角度重新决策
#ifdef OBJECT_FOLLOWER_DEBUG
                    Serial.print("[前进检查] 稳定后发现偏离，角度：");
                    Serial.print(stable_angle);
                    Serial.println("，将重新决策转向");
#endif
                    target_angle = stable_angle;
                    resetStableCheck();
                    state = FollowState::DECIDE;
                }
            }
            break;
        }

        // ================= 前进完成 =================
        case FollowState::FORWARD_DONE: {
            if (millis() - step_timer > POST_FORWARD_PAUSE) {
#ifdef OBJECT_FOLLOWER_DEBUG
                Serial.println("[前进完成] 停顿结束，头部回中，重新观察");
#endif
                headTracker_resetToCenter();   // 强制头部回到 90°，重新寻找物体
                resetStableCheck();
                stable_block_time = millis();
                state = FollowState::WAIT_STABLE;
            }
            break;
        }

        // ================= 已到达 =================
        case FollowState::ARRIVED: {
            if (!arrived_executed) {
                headTracker_resetToCenter();    // 头部回中
                Controller.runActionGroup(0, 1); // 执行0号动作组（直立）
                arrived_executed = true;
#ifdef OBJECT_FOLLOWER_DEBUG
                Serial.println("[已到达] 头部回中，执行直立动作");
#endif
            }
            break;  // 停留在此状态，不再动作
        }
    }
}