#ifndef USER_H
#define USER_H
#include <stdio.h>
#include "ti_msp_dl_config.h"

// ---------- 串口通用函数 ----------
void UART_send_byte(UART_Regs *uart, uint8_t data);
void UART_send_buffer(UART_Regs *uart, uint8_t* buf, uint16_t len);
void UART_send_int(UART_Regs *uart, int value);
void UART_send_float(UART_Regs *uart, float value);
void UART_send_string(UART_Regs *uart, const char* str);
#endif