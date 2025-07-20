#include "user.h"
#include "string.h"

// ---------- 串口通用函数 ----------
void UART_send_byte(UART_Regs *uart, uint8_t data) {
    while (DL_UART_isTXFIFOFull(uart)) {
    }
    DL_UART_transmitData(uart, data);   
}

// 只打印buf 里面的有效值，不会新加空格
void UART_send_buffer(UART_Regs *uart, uint8_t* buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        UART_send_byte(uart, buf[i]);
    }
}

void UART_send_int(UART_Regs *uart, int value) {
    char buffer[16];  // 足够容纳 int 的字符串形式
    snprintf(buffer, sizeof(buffer), "%d", value);  // 将整数转换为字符串
    UART_send_buffer(uart, (uint8_t*)buffer, strlen(buffer));  // 发送字符串
    UART_send_byte(uart, ' ');  // 添加一个空格
}

// 打印一个 float 类型数值（保留 2 位小数）
void UART_send_float(UART_Regs *uart, float value) {
    char buffer[32];  // 足够容纳 float 的字符串形式
    snprintf(buffer, sizeof(buffer), "%.2f", value);  // 转换为字符串，保留两位小数
    UART_send_buffer(uart, (uint8_t*)buffer, strlen(buffer));  // 通过串口发送
    UART_send_byte(uart, ' ');  // 添加一个空格
}

// 发送字符串函数
void UART_send_string(UART_Regs *uart, const char* str) {
    while (*str != '\0') {
        UART_send_byte(uart, (uint8_t)(*str));
        str++;
    }
}