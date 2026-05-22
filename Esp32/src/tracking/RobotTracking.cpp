#include "RobotTracking.h"
#include "../Servo.h"
#include "../LobotServoController.h"

extern Servo headServo; 
extern LobotServoController Controller;

// ==========================================
// 1. 替换为真实的动作组 ID 
// ==========================================
#define ACTION_FORWARD    21 // 直走
#define ACTION_LEFT_TURN  23 // 左转
#define ACTION_RIGHT_TURN 24 // 右转

// ==========================================
// 2. 状态记忆变量 
// ==========================================
static float head_pan_angle = 90.0; 
static bool is_action_running = false;
static unsigned long action_start_time = 0;
// 根据你们机器人的步态速度，稍微留一点动作余量，比如 1500 毫秒
static unsigned long action_duration = 1500; 

// ==========================================
// 3. 填满硬件接口封装层（肌肉对接完毕！）
// ==========================================
void set_head_servo(int angle) {
    // 真实调用：让脖子舵机转到指定角度
    headServo.write(angle);
}

void trigger_action(int action_id) {
    // 真实调用：执行动作组 1 次
    Controller.runActionGroup(action_id, 1);
    
    Serial.print(">> [硬件层] 正在执行步态: ");
    Serial.println(action_id);
}

// ... [下方的 init_tracking 和 execute_pid_tracking 保持不变] ...

// ==========================================
// 4. 对外暴露的 API
// ==========================================
void init_tracking() {
    head_pan_angle = 90.0;
    is_action_running = false;
    set_head_servo((int)head_pan_angle);
    Serial.println("追踪模块初始化完毕，头部回正！");
}

void execute_pid_tracking(uint16_t x, uint8_t y) {
    // ---------------------------------------------------------
    // 第一层：软追踪（只控制脖子，极度丝滑）
    // ---------------------------------------------------------
    // 计算屏幕中心偏差 (假设画面宽 320，中心是 160)
    int error_x = x - 160; 
    
    // 比例系数 (Kp)：调教脖子扭动速度。值太小扭得慢，太大容易摇头晃脑
    float Kp = 0.05; 
    
    // 计算新角度。注意：这里的减号(-)如果导致脖子反向扭，请把它改成加号(+)
    head_pan_angle = head_pan_angle - (error_x * Kp); 
    
    // 硬件限位保护（千万别把机器人脖子拧断了！）
    if (head_pan_angle > 150.0) head_pan_angle = 150.0;
    if (head_pan_angle < 30.0) head_pan_angle = 30.0;
    
    // 每次收到坐标，脖子立刻微调！
    set_head_servo((int)head_pan_angle);

    // ---------------------------------------------------------
    // 【架构师核心设计】：非阻塞计时器（代替 delay）
    // ---------------------------------------------------------
    if (is_action_running) {
        // 如果当前时间 减去 开始时间 大于 动作耗时，说明动作做完了
        if (millis() - action_start_time > action_duration) {
            is_action_running = false; // 解除动作锁定
        } else {
            return; // 动作还没做完，直接退出函数，不执行下面的腿部逻辑
        }
    }

    // ---------------------------------------------------------
    // 第二层：硬追踪（根据脖子朝向，控制腿部动作组）
    // ---------------------------------------------------------
    if (!is_action_running) {
        if (head_pan_angle > 110.0) {
            trigger_action(ACTION_LEFT_TURN);
            is_action_running = true;
            action_start_time = millis(); // 记录动作开始的时间戳
            
        } else if (head_pan_angle < 70.0) {
            trigger_action(ACTION_RIGHT_TURN);
            is_action_running = true;
            action_start_time = millis();
            
        } else {
            // 脖子在 70~110 度之间，说明球在正前方死区，大胆往前走！
            trigger_action(ACTION_FORWARD);
            is_action_running = true;
            action_start_time = millis();
        }
    }
}