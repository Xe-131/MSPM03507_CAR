#include "motor.h"

// 所有电机的standby 是否关闭
uint8_t motor_on_flag   = 0;

/**
 * @brief   关闭所有电机的standby
 * @param   无
 * @param   无
 * @return  无
 */
void Motor_On(void)
{
    motor_on_flag   = 1;
    DL_GPIO_setPins(GPIO_MOTOR_PIN_LEFT_STSNDBY_PORT, GPIO_MOTOR_PIN_LEFT_STSNDBY_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_PIN_RIGHT_STSNDBY_PORT, GPIO_MOTOR_PIN_RIGHT_STSNDBY_PIN);
}

/**
 * @brief   打开所有电机的standby
 * @param   无
 * @param   无
 * @return  无
 */
void Motor_Off(void)
{
    motor_on_flag   = 0;
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_LEFT_STSNDBY_PORT, GPIO_MOTOR_PIN_LEFT_STSNDBY_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_PIN_RIGHT_STSNDBY_PORT, GPIO_MOTOR_PIN_RIGHT_STSNDBY_PIN);
}


/**
 * @brief 为单侧轮子设置PWM duty
 * @param side 宏
 * @param duty 范围在-100 到 100，为float 类型
 * @return 无
 * @note 超过范围的duty 取-100或者100
 */
void Set_Duty(uint8_t side, float duty)
{
    uint32_t compareValue   = 0;
    uint32_t loadvalue      = 4000 - 1;

    // 限制duty 范围
    if(duty < -100){
        duty    = -100;
    }
    else if(duty > 100){
        duty    = 100;
    }

    if(side == LEFT){
        if((duty <= 0) && (duty >= -100))
        {
            compareValue = loadvalue - loadvalue * (-duty/100.0);
            // PWM
            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_2_INDEX);
            // 方向
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_LEFT_DIRECT_1_PORT, GPIO_MOTOR_PIN_LEFT_DIRECT_1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_LEFT_DIRECT_2_PORT, GPIO_MOTOR_PIN_LEFT_DIRECT_2_PIN);
        }
        else if((duty >= 0) && (duty <= 100))
        {
            compareValue = loadvalue - loadvalue * (duty/100.0);
            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_2_INDEX);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_LEFT_DIRECT_1_PORT, GPIO_MOTOR_PIN_LEFT_DIRECT_1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_LEFT_DIRECT_2_PORT, GPIO_MOTOR_PIN_LEFT_DIRECT_2_PIN);

        }
        // 非法duty --- 关闭电机 standby
        else 
        {
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_LEFT_STSNDBY_PORT, GPIO_MOTOR_PIN_LEFT_STSNDBY_PIN);
        }
    }
    else if(side == RIGHT){
        if((duty <= 0) && (duty >= -100))
        {
            compareValue = loadvalue - loadvalue * (-duty/100.0);
            // PWM
            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_3_INDEX);
            // 方向
            DL_GPIO_setPins(GPIO_MOTOR_PIN_RIGHT_DIRECT_1_PORT, GPIO_MOTOR_PIN_RIGHT_DIRECT_1_PIN);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_RIGHT_DIRECT_2_PORT, GPIO_MOTOR_PIN_RIGHT_DIRECT_2_PIN);
        }
        else if((duty >= 0) && (duty <= 100))
        {
            compareValue = loadvalue - loadvalue * (duty/100.0);
            DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, compareValue, DL_TIMER_CC_3_INDEX);
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_RIGHT_DIRECT_1_PORT, GPIO_MOTOR_PIN_RIGHT_DIRECT_1_PIN);
            DL_GPIO_setPins(GPIO_MOTOR_PIN_RIGHT_DIRECT_2_PORT, GPIO_MOTOR_PIN_RIGHT_DIRECT_2_PIN);

        }
        // 非法duty --- 关闭电机 standby
        else 
        {
            DL_GPIO_clearPins(GPIO_MOTOR_PIN_RIGHT_STSNDBY_PORT, GPIO_MOTOR_PIN_RIGHT_STSNDBY_PIN);
        }
    }
}