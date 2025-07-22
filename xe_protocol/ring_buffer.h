#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
// 定义环形缓冲区的结构体
typedef struct {
    uint8_t *buffer;    // 指向实际存储数据的数组
    size_t   size;      // 缓冲区总大小
    size_t   head;      // 头指针（读取位置）
    size_t   tail;      // 尾指针（写入位置）
    bool     is_full;   // 缓冲区是否已满的标志
} RingBuffer_t;

// 函数原型
void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, size_t size);
bool RingBuffer_Write(RingBuffer_t *rb, uint8_t byte);
bool RingBuffer_Read(RingBuffer_t *rb, uint8_t *byte);
bool RingBuffer_IsEmpty(const RingBuffer_t *rb);

// 实际用到的缓冲区
#define UART_PC_BUFFER_SIZE 50
extern uint8_t uart_pc_rx_buffer_data[UART_PC_BUFFER_SIZE];
extern RingBuffer_t uart_pc_rx_buffer;

#define UART_MAVLINK_BUFFER_SIZE 600
extern uint8_t uart_mavlink_rx_buffer_data[UART_MAVLINK_BUFFER_SIZE];
extern RingBuffer_t uart_mavlink_rx_buffer;

#endif // RING_BUFFER_H