#ifndef __MOTOR_h_
#define __MOTOR_h_

#include "ti_msp_dl_config.h"

#define LEFT    0
#define RIGHT   1

extern uint8_t motor_on_flag;

void Motor_On(void);
void Motor_Off(void);
void Set_Duty(uint8_t side, float duty);

#endif