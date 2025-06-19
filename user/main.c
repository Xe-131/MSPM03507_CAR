#include "ti_msp_dl_config.h"
#include "motor.h"
#include "pid.h"
#include "user.h"
#include "oled_hardware_i2c.h"
#include "clock.h"
// #include "mpu6050.h"

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

    // systick
    SysTick_Init();
    // OLED
    OLED_Init();
    OLED_ShowString(0,4,(uint8_t *)"  Yaw",16);
    // uint8_t oled_buffer[32];
    // MPU6050
    // MPU6050_Init();

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

        // // 显示YAW 角
        // sprintf((char *)oled_buffer, "%-6.1f", yaw);
        // OLED_ShowString(5*8,4,oled_buffer,16);
    }
}


