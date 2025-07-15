#ifndef USER_H
#define USER_H
#include <stdio.h>
#include "ti_msp_dl_config.h"

// 纸飞机绘图用
void datavision_send(float now_speed_left, float target_speed_left, float now_speed_right, float target_speed_right);
// 发送字符串
void UART_sendString(char *str);
void UART_send(uint8_t byte);

// 通用
void UART_send_byte(UART_Regs *uart, uint8_t data);
void UART_send_buffer(UART_Regs *uart, uint8_t* buf, uint16_t len);
#endif