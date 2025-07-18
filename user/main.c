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
    pid_init(&pid_distance, DELTA_PID, 0.1, 0.005, 0);
    set_target_angle(0);
    set_target_speed(0);



    while (1) {
        // 调试
        // oled_display(usart_buffer);
        // sprintf(usart_buffer, "angle: %f angle_init: %f\r\n", angle, angle_init);
        // UART_sendString(usart_buffer);
        // 绘图
	    // datavision_send(now_speed_left, target_speed_left, now_speed_right, target_speed_right);
        // sprintf(usart_buffer, "left duty: %f right duty: %f angleduty: %f target_angle: %f\r\n", pid_motor_left.out, pid_motor_right.out, pid_angle.out, target_angle);
        // UART_sendString(usart_buffer);	

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

            // 开启路径规划
            set_target_point(150, -150);
            Point_t path_test[] = {
                {0.0f, 560.0f},  // 第1个点
                {240.0f, 540.0f},  // 第2个点
                {250.0f, 225.0f},  // 第3个点
                {460.0f, 240.0f},   // 第4个点，回到原点
                {453.0f, -160.0f},  // 第2个点
                {125.0f, -150.0f},  // 第3个点
                {-30.0f, 90.0f}   // 第4个点，回到原点
            };
            path_start(path_test, 7);
        }

        // PID 计算
        if(pid_timer_flag == 1){
            pid_timer_flag    = 0;

            pid_controal();            
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
            UART_send_float(UART_BLUEUART_INST, NOW_z);
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



