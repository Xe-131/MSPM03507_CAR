#include "user.h"

// 纸飞机绘图用
void datavision_send(float now_speed_left, float target_speed_left, float now_speed_right, float target_speed_right){
    char buffer[64];  // 足够大，避免溢出

    // 将 int 数据插入到字符串中
    sprintf(buffer, "{plotter:%f,%f,%f,%f}\r\n", now_speed_left, target_speed_left, now_speed_right, target_speed_right);

    // 发送拼接好的字符串
    UART_sendString(buffer);
}

// 发送字符串
void UART_sendString(char *str)
{
    while (*str) {
        while (DL_UART_isTXFIFOFull(UART_PC_INST)) {
        }
        DL_UART_transmitData(UART_PC_INST, (uint8_t)(*str));
        str++;  // 下一个字符
    }
}