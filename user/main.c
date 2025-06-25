#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "user.h"
#include "oled_hardware_i2c.h"
#include "clock.h"
#include "mpu6050.h"
#include "interrupt.h"


int main(void){
    char usart_buffer[200];
    
    SYSCFG_DL_init();

    // delay_cycles(CPUCLK_FREQ);
    // UART_sendString("12345");
    // 开启测速的外部中断
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_RIGHT_INT_IRQN);
    // 开启TIMER_PID 中断
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);
    // TIMER_INTOWHILE 中断
    NVIC_EnableIRQ(TIMER_INTOWHILE_INST_INT_IRQN);
    // TIMER_GENERAL 中断
    NVIC_EnableIRQ(TIMER_GENERAL_INST_INT_IRQN);

    // systick
    SysTick_Init();
    
    // OLED
    // OLED_Init();
    // OLED_ShowString(0,2,(uint8_t *)"  Yaw",16);

    // // MPU6050
    MPU6050_Init();
    // TIMER_INTOWHILE 开始计时
    DL_Timer_setLoadValue(TIMER_INTOWHILE_INST, TIMER_INTOWHILE_PERIOD / 10.0);
    DL_TimerG_startCounter(TIMER_INTOWHILE_INST);

    // // PID
    pid_init(&pid_motor_left, DELTA_PID, 0.1, 0.005, 0.00);
    pid_init(&pid_motor_right, DELTA_PID, 0.1, 0.001, 0.00);
    pid_init(&pid_angle, POSITION_PID, 20, 0.005, 0);

    // 电机初始化
    Set_Duty(RIGHT, 0);
    Set_Duty(LEFT, 0);
    Motor_Off();
    while (1) {
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

        // 调试
        // // 显示YAW 角
        // sprintf((char *)oled_buffer, "%-6.1f", yaw);
        // OLED_ShowString(6*6,2,oled_buffer,16);
        // sprintf((char *)oled_buffer, "%-6.1f", yaw_init);
        // OLED_ShowString(6*6,5,oled_buffer,16);
        sprintf(usart_buffer, "angle: %f angle_init: %f\r\n", angle, angle_init);
        UART_sendString(usart_buffer);
        // 绘图
	    // datavision_send(now_speed_left, target_speed_left, now_speed_right, target_speed_right);
        sprintf(usart_buffer, "left duty: %f right duty: %f angleduty: %f target_angle: %f\r\n", pid_motor_left.out, pid_motor_right.out, pid_angle.out, target_angle);
        UART_sendString(usart_buffer);	

        // yaw 角稳定后
        if(into_wihle_flag == 1){
            into_wihle_flag = 0;

            angle_init        = angle;
            reset_pid(&pid_motor_left);
            reset_pid(&pid_motor_right);
            reset_pid(&pid_angle);
            Motor_On();

            // TIMER_GENERAL
            DL_Timer_setLoadValue(TIMER_GENERAL_INST, TIMER_GENERAL_PERIOD / 10.0);
            DL_Timer_startCounter(TIMER_GENERAL_INST);

            // TIMER_PID 
            DL_Timer_setLoadValue(TIMER_PID_INST, TIMER_PID_PERIOD / 10.0);
            DL_TimerG_startCounter(TIMER_PID_INST);
        }

        // PID 计算
        if(pid_timer_flag == 1){
            pid_timer_flag    = 0;

            pid_controal();            
        }

        // 通用定时
        if(general_timer_flag == 1){
            general_timer_flag    = 0;

            // target_angle += 180;
            // if(target_angle >= 360){
            //     target_angle -= 360;
            // }        
        }

    }
}


