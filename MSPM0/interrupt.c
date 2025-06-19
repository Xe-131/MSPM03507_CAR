#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "motor.h"
#include "pid.h"
#include "user.h"
#include "mpu6050.h"

// 关于PID 的速度全局变量定义在main.c 中
extern int encode_cnt_left;
extern int encode_cnt_right;
// 实时速度 mm/s
extern float now_speed_left;
extern float now_speed_right;
// 目标速度
extern float target_speed_left;
extern float target_speed_right;

// SysTick 中断，每隔1ms 发送一次
void SysTick_Handler(void)
{
    tick_ms++;
}

void GROUP1_IRQHandler(void)
{
    uint32_t gpioA  = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_ENCODER_RIGHT_PIN_EA_RIGHT_PIN);
    uint32_t gpioB  = DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_ENCODER_LEFT_PIN_EA_LEFT_PIN | GPIO_MPU6050_PIN_INT_PIN);

    // 编码器right 中断
    if(gpioA & GPIO_ENCODER_RIGHT_PIN_EA_RIGHT_PIN){
        if (!DL_GPIO_readPins(GPIO_ENCODER_RIGHT_PIN_EB_RIGHT_PORT, GPIO_ENCODER_RIGHT_PIN_EB_RIGHT_PIN)) {
            encode_cnt_right--;   
        }
        else {
            encode_cnt_right++;   
        }
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_ENCODER_RIGHT_PIN_EA_RIGHT_PIN);
    }

    // 编码器left 中断
    if(gpioB & GPIO_ENCODER_LEFT_PIN_EA_LEFT_PIN){
        if (!DL_GPIO_readPins(GPIO_ENCODER_LEFT_PIN_EB_LEFT_PORT, GPIO_ENCODER_LEFT_PIN_EB_LEFT_PIN)) {
            encode_cnt_left++;   
        }
        else {
            encode_cnt_left--; 
        }
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_ENCODER_LEFT_PIN_EA_LEFT_PIN);
    }

    // MPU6050 中断
    if(gpioB & GPIO_MPU6050_PIN_INT_PIN){
        Read_Quad();
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_MPU6050_PIN_INT_PIN);
    }

}

// TIMA1
// 定时清空编码器计数器，并计算实时速度
// ms
# define TIMER_PID_PERIOD 100
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

                // char buffer[64];   // 足够大，避免溢出
                // // 将 int 数据插入到字符串中
                // sprintf(buffer, "pid->out: %f speed: %f\r\n", pid_motor_right.out, now_speed_right);
                // // 发送拼接好的字符串
                // UART_sendString(buffer);
            break;
        default:
            break;
    }
}