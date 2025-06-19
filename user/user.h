#ifndef USER_H
#define USER_H
#include <stdio.h>
#include "ti_msp_dl_config.h"

// 纸飞机绘图用
void datavision_send(float now_speed_left, float target_speed_left, float now_speed_right, float target_speed_right);
// 发送字符串
void UART_sendString(char *str);

#endif