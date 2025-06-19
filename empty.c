/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "user.h"
#include "oled_hardware_i2c.h"

// ms
# define TIMER_PID_PERIOD 100
// 编码器计数
int encode_cnt_left     = 0;
int encode_cnt_right    = 0;
// 实时速度 mm/s
float now_speed_left    = 0;
float now_speed_right   = 0;
// 目标速度
float target_speed_left     = 500;
float target_speed_right    = 500;


int main(void){
    SYSCFG_DL_init();
    // 开启测速的外部中断
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_ENCODER_RIGHT_INT_IRQN);
    // 开启TIMER_PID 中断
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);

    // OLED
    SysTick_Init();
    OLED_Init();
    OLED_ShowString(0,7,(uint8_t *)"MPU6050 Demo",8);
    OLED_ShowString(0,0,(uint8_t *)"Pitch",8);
    OLED_ShowString(0,2,(uint8_t *)" Roll",8);
    OLED_ShowString(0,4,(uint8_t *)"  Yaw",8);
    OLED_ShowString(16*6,3,(uint8_t *)"Accel",8);
    OLED_ShowString(17*6,4,(uint8_t *)"Gyro",8);

    // PID
    pid_init(&pid_motor_left, DELTA_PID, 0.115, 0.01, 0.01);
    pid_init(&pid_motor_right, DELTA_PID, 0.115, 0.01, 0.01);

    delay_cycles(CPUCLK_FREQ);
    Set_Duty(RIGHT, 0);
    Set_Duty(LEFT, 0);
    Motor_On();


    delay_cycles(CPUCLK_FREQ);
    delay_cycles(CPUCLK_FREQ);
    delay_cycles(CPUCLK_FREQ);
    delay_cycles(CPUCLK_FREQ);
    Motor_Off();
    while (1) {
       

        if(!DL_GPIO_readPins(GPIO_SWICH_PORT, GPIO_SWICH_PIN_S2_PIN)){
            delay_cycles(CPUCLK_FREQ/2);
            Motor_Off();
        }  
    }
}


// 编码器中断
void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        // left
        case GPIO_MULTIPLE_GPIOB_INT_IIDX:
            if (!DL_GPIO_readPins(GPIO_ENCODER_LEFT_PIN_EB_LEFT_PORT, GPIO_ENCODER_LEFT_PIN_EB_LEFT_PIN)) {
                encode_cnt_left++;   
            }
            else {
                encode_cnt_left--; 
            }
            break;
        // right
        case GPIO_ENCODER_RIGHT_INT_IIDX:
            if (!DL_GPIO_readPins(GPIO_ENCODER_RIGHT_PIN_EB_RIGHT_PORT, GPIO_ENCODER_RIGHT_PIN_EB_RIGHT_PIN)) {
                encode_cnt_right--;   
            }
            else {
                encode_cnt_right++;   
            }
            break;
    }
}

// TIMA1
// 定时清空编码器计数器，并计算实时速度
void TIMER_PID_INST_IRQHandler(void){
    switch (DL_TimerG_getPendingInterrupt(TIMER_PID_INST)) {
        // 清零中断位
        case DL_TIMER_IIDX_ZERO:
                DL_TimerG_stopCounter(TIMER_PID_INST);
                DL_Timer_setLoadValue(TIMER_PID_INST, TIMER_PID_PERIOD / 10.0);
                DL_TimerG_startCounter(TIMER_PID_INST);

                // 计算转速              
                // mm/s
                now_speed_left  = ((encode_cnt_left*(1.0)) / TIMER_PID_PERIOD) * 1000 / 260 * 150.79;
                now_speed_right = ((encode_cnt_right*(1.0)) / TIMER_PID_PERIOD) * 1000 / 260 * 150.79;
                // 归零计数器
                encode_cnt_left     = 0;
                encode_cnt_right    = 0;
                // 设置下一个duty
                Set_Duty(LEFT, (int8_t)pid_calculate(&pid_motor_left, now_speed_left, target_speed_left));
                Set_Duty(RIGHT, (int8_t)pid_calculate(&pid_motor_right, now_speed_right, target_speed_right));
                // 绘图
                datavision_send(now_speed_left, target_speed_left, now_speed_right, target_speed_right);

                char buffer[64];   // 足够大，避免溢出
                // 将 int 数据插入到字符串中
                sprintf(buffer, "pid->out: %f speed: %f\r\n", pid_motor_right.out, now_speed_right);
                // 发送拼接好的字符串
                UART_sendString(buffer);

            break;
        default:
            break;
    }

}