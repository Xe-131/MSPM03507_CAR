#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "user.h"
#include "clock.h"
#include "mpu6050.h"
#include "interrupt.h"
#include "uwb.h"
#include "mavlink.h"
#include "position.h"

bool run_all_paths_continuously(void);

int main(void){
    uint8_t usart_buffer[200];
    
    SYSCFG_DL_init();
    // 开启测速的外部中断
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_RIGHT_INT_IRQN);
    // 开启TIMER_PID 中断
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);
    // TIMER_INTOWHILE 中断
    NVIC_EnableIRQ(TIMER_INTOWHILE_INST_INT_IRQN);
    // TIMER_GENERAL 中断
    NVIC_EnableIRQ(TIMER_GENERAL_INST_INT_IRQN);
    // TIMER_GENERAL 开始计时
    DL_Timer_setLoadValue(TIMER_GENERAL_INST, TIMER_GENERAL_PERIOD / 10.0);
    DL_Timer_startCounter(TIMER_GENERAL_INST);
    // UART_MAVLINK 中断
    NVIC_ClearPendingIRQ(UART_MAVLINK_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_MAVLINK_INST_INT_IRQN);

    // systick
    SysTick_Init();
    
    // MPU6050
    MPU6050_Init();
    // TIMER_INTOWHILE 开始计时
    DL_Timer_setLoadValue(TIMER_INTOWHILE_INST, TIMER_INTOWHILE_PERIOD / 10.0);
    DL_TimerG_startCounter(TIMER_INTOWHILE_INST);

    // 电机初始化
    Set_Duty(RIGHT, 0);
    Set_Duty(LEFT, 0);
    Motor_Off();

    // PID
    pid_init(&pid_motor_left, DELTA_PID, 0.1, 0.005, 0.00);
    pid_init(&pid_motor_right, DELTA_PID, 0.1, 0.001, 0.00);
    pid_init(&pid_angle, POSITION_PID, 35, 0.005, 0);
    //
    pid_init(&pid_distance, DELTA_PID, -6, 0.005, 0);
    set_target_angle(0);
    set_target_speed(0);


    while (1) {

        
        
        
        
        // UWB 解析坐标
        mavlink_decode_receive_message();
        
        // 开关电机
        if(S2_flag == 1){
            S2_flag = 0;

            if(motor_on_flag){
                Motor_Off();
            }
            else{
                Motor_On();
            }
        }  

        // yaw 角稳定后
        if(into_wihle_flag == 1){
            into_wihle_flag = 0;

            angle_init        = angle;
            reset_pid(&pid_motor_left);
            reset_pid(&pid_motor_right);
            reset_pid(&pid_angle);
            Motor_On();

            // TIMER_PID 
            DL_Timer_setLoadValue(TIMER_PID_INST, TIMER_PID_PERIOD / 10.0);
            DL_TimerG_startCounter(TIMER_PID_INST);

            //
            // diff_x = 30 - uwb.x;
            // diff_y = 130 - uwb.y;
            // // 开启路径规划
            // path_start(path_5, 4);
        }

        // PID 计算
        if(pid_timer_flag == 1){
            pid_timer_flag    = 0;

            pid_controal();  


        // // --- 只需在循环中调用这一个函数 ---
        run_all_paths_continuously();          
        }

        // 通用定时

        if(flag_100ms){
            flag_100ms  = 0;
            
            // 单点导航更新
            navigation_update();
            // 多点路径更新
            path_update();
        }

        if(flag_1s){
            flag_1s  = 0;

            UART_send_string(UART_BLUEUART_INST, "coordinate: \r\n");
            UART_send_float(UART_BLUEUART_INST, NOW_x);
            UART_send_float(UART_BLUEUART_INST, NOW_y);
            UART_send_string(UART_BLUEUART_INST, "\r\nstate: \r\n");
            if(nav_state == NAV_IDLE){
                UART_send_int(UART_BLUEUART_INST, 11111);
            }
            else if(nav_state == NAV_ROTATING){
                UART_send_int(UART_BLUEUART_INST, 22222);
            }
            else if(nav_state == NAV_MOVING){
                UART_send_int(UART_BLUEUART_INST, 33333);
            }
            else if(nav_state == NAV_ARRIVED){
                UART_send_int(UART_BLUEUART_INST, 44444);
            }
            UART_send_string(UART_BLUEUART_INST, "\r\nSPEED: \r\n");
            UART_send_float(UART_BLUEUART_INST, target_speed_left);
        }
        
    }
}



/*******************************************************************************
 *         单函数任务序列管理器 (12条路径连续无暂停版)
 ******************************************************************************/

/**
 * @brief 连续执行12条预设路径（正向+反向），中间无任何暂停。
 * @return bool 如果所有12条路径都已完成，则返回 true；否则返回 false。
 * @note 此函数应在主循环中以固定频率被持续调用。它内部管理所有状态，
 *       第一次被调用时会自动开始任务。
 */
bool run_all_paths_continuously(void) {
    // 使用 static 变量来保存函数在多次调用之间的状态（记忆）
    static enum {
        MISSION_IDLE,       // 0: 空闲，等待第一次调用
        MISSION_RUNNING,    // 1: 正在执行路径序列
        MISSION_COMPLETE    // 2: 所有任务已完成
    } mission_state = MISSION_IDLE;

    static int current_path_index = 0; // 当前正在执行第几条路径 (0-11)

    // --- 创建一个包含所有12条路径的“主播放列表” ---
    static const Point_t* master_path_list[] = {
        path_1, reverse_path_1,
        path_2, reverse_path_2,
        path_3, reverse_path_3,
        path_4, reverse_path_4,
        path_5, reverse_path_5,
        path_6, reverse_path_6
    };

    static const int master_path_sizes[] = {
        2,2,2,2,2,2,4,4,4,4,4,4
    };
    const int TOTAL_MASTER_PATHS = 12;

    // 状态机核心逻辑
    switch (mission_state) {
        case MISSION_IDLE:
            // 第一次调用时，自动开始任务
            current_path_index = 0;
            path_start(master_path_list[current_path_index], master_path_sizes[current_path_index]);
            mission_state = MISSION_RUNNING;
            break;

        case MISSION_RUNNING:
            // 检查当前路径是否完成
            if (path_is_finished()) {
                // 当前路径完成，立即准备下一个
                current_path_index++;
                if (current_path_index < TOTAL_MASTER_PATHS) {
                    // 如果列表里还有路径，则开始它
                    path_start(master_path_list[current_path_index], master_path_sizes[current_path_index]);
                    // 状态保持为 MISSION_RUNNING
                } else {
                    // 所有路径都已完成
                    mission_state = MISSION_COMPLETE;
                }
            }
            break;

        case MISSION_COMPLETE:
            // 所有任务已完成，保持此状态。
            break;
    }
    
    // 只要任务正在进行，就必须持续调用底层的导航更新函数
    if (mission_state == MISSION_RUNNING) {
        navigation_update();
    }

    // 返回任务是否已完成
    return (mission_state == MISSION_COMPLETE);
}