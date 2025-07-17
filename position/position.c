#include "position.h"
#include "math.h"
#include "pid.h"
#include "mpu6050.h"
#include "user.h"
#include "pid.h"

// UWB 实时坐标
float NOW_x;
float NOW_y;
float NOW_z;

// 目的坐标
float TARGET_x;
float TARGET_y;
float TARGET_z;

// 导航状态
NavigationState_t nav_state = NAV_IDLE;

// ---------- 公共接口函数 ----------

/**
 * @brief   
 * @param   
 * @param   
 * @param   
 * @return  
 * @note 
 */
void set_target_point(float x, float y){
    TARGET_x    = x;
    TARGET_y    = y;
    nav_state   = NAV_ROTATING;
}

/**
 * @brief   
 * @param   
 * @param   
 * @param   
 * @return  
 * @note 
 */
 void navigation_update(void) {
    // 角度稳定计数
    static int rotation_stable_counter = 0;
    // 距离稳定计数
    static int distance_stable_counter = 0;

    // 计算当前位置到目标的向量和距离
    float dx = TARGET_x - NOW_x;
    float dy = TARGET_y - NOW_y;
    float distance_to_target = sqrtf(dx * dx + dy * dy);

    // 状态机核心逻辑
    switch (nav_state) {

        case NAV_IDLE:
        // 调试
        UART_send_string(UART_BLUEUART_INST, "STATE:");
        UART_send_int(UART_BLUEUART_INST, 11111);

            // 空闲状态，什么也不做，确保小车停止
            set_target_speed(0);
            set_target_angle(now_angle);
            break;

        case NAV_ROTATING: {
        // 调试
        UART_send_string(UART_BLUEUART_INST, "STATE:");
        UART_send_int(UART_BLUEUART_INST, 22222);    

            // --- 目标: 原地旋转对准目标点 ---
            
            // 1. 计算目标角度:这里注意坐标系，xy还是yx
            float math_angle_deg = atan2f(dx, dy) * 180.0f / M_PI;
            float angle_to_be = normalize_angle(math_angle_deg - 90.0f);
            
            // 2. 设置电机，速度为0，目标为计算出的角度
            set_target_speed(0);
            set_target_angle(angle_to_be);
            
            // 3. 检查是否对准
            float angle_error = angle_to_be - now_angle;
            // 处理 359 -> 1 度的角度跳变问题
            if (angle_error > 180.0f) angle_error -= 360.0f;
            if (angle_error < -180.0f) angle_error += 360.0f;

            if (fabsf(angle_error) < ANGLE_THRESHOLD) {
                // 如果角度在阈值内，增加稳定计数器
                rotation_stable_counter++;
            } else {
                // 如果角度超出阈值，立即清零计数器，因为稳定被打破
                rotation_stable_counter = 0;
            }

            // 4. 检查稳定计数器是否达到目标帧数
            if (rotation_stable_counter >= ROTATION_STABLE_FRAMES) {
                // 稳定时间足够长，可以安全地切换到前进状态
                nav_state = NAV_MOVING;     
                rotation_stable_counter = 0;       
            }
            break;
        }

        case NAV_MOVING: {
        // 调试
        UART_send_string(UART_BLUEUART_INST, "STATE:");
        UART_send_int(UART_BLUEUART_INST, 33333);

            // --- 目标: 朝着目标点前进，并实时修正方向 ---
            
            // 1. 检查是否已到达
            if (distance_to_target < DISTANCE_THRESHOLD) {
                distance_stable_counter++;
            } else {
                distance_stable_counter = 0;
            }
            if (distance_stable_counter >= DISTANCE_STABLE_FRAMES) {
                nav_state = NAV_ARRIVED;     
                distance_stable_counter = 0;       
            }
            // 2. 实时计算并更新目标角度，用于航向保持
            float math_angle_deg = atan2f(dx, dy) * 180.0f / M_PI;
            float target_angle = normalize_angle(math_angle_deg - 90.0f);
            set_target_angle(target_angle);
            
            // 3. 根据距离动态计算前进速度 (P控制器)
            // float desired_speed = distance_to_target * KP_DISTANCE_TO_SPEED;
            
            // 4. 对速度进行限幅
            // desired_speed = limit_float(desired_speed, 0, MAX_FORWARD_SPEED);
            
            // 5. 设置目标速度
            set_target_speed(100);
            
            break;
        }

        case NAV_ARRIVED:
        // 调试
        UART_send_string(UART_BLUEUART_INST, "STATE:");
        UART_send_int(UART_BLUEUART_INST, 44444);
            // --- 目标: 停止小车 ---
            set_target_speed(0);
            // 可以在这里设置目标角度为当前角度，防止漂移
            set_target_angle(now_angle);
            
            // 转换到空闲状态，等待下一个指令
            nav_state = NAV_IDLE;
            break;
    }
}

// ---------- 内部辅助函数 ----------

/**
 * @brief 将一个任意角度值标准化到 [0, 360) 的范围内
 * @param angle 输入的角度，可以是负数或大于360的数
 * @return 标准化后的角度，范围在 [0, 360)
 */
static float normalize_angle(float angle){
    // 如果角度是负数，则不断加360直到它变为正数
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    // 如果角度大于等于360，则不断减360直到它小于360
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}