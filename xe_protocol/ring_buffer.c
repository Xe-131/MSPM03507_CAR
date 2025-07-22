#include "ring_buffer.h"
#include "user.h"
#include "ti_msp_dl_config.h"

/*
串口缓冲区使用：
    1.在此文件以及此文件的头文件内声明或者定义好缓冲区大小，结构体，缓冲区
    2.在主函数中初始化调用函数RingBuffer_Init() 进行初始化
    ---此时缓冲区就以及准备好了，要与串口相关联就得执行下面的步骤---
    3.在某个串口的接收中断中读出字节，并调用RingBuffer_Write() 写入缓冲区
    4.在主循环中持续读取缓冲区字节，并处理：
                if (RingBuffer_Read(&uart_mavlink_rx_buffer, &byte_from_buffer)) {
                    // 处理读到字节
                    handle_byte_user(byte);
                }
*/

// 实际用到的缓冲区
// #define UART_PC_BUFFER_SIZE 50
uint8_t uart_pc_rx_buffer_data[UART_PC_BUFFER_SIZE];
RingBuffer_t uart_pc_rx_buffer;
// #define UART_MAVLINK_BUFFER_SIZE 50
uint8_t uart_mavlink_rx_buffer_data[UART_MAVLINK_BUFFER_SIZE];
RingBuffer_t uart_mavlink_rx_buffer;

// 初始化环形缓冲区
void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, size_t size) {
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->is_full = false;
}

// 向缓冲区写入一个字节（由中断服务程序调用）
bool RingBuffer_Write(RingBuffer_t *rb, uint8_t byte) {
    if (rb->is_full) {
        return false; // 缓冲区已满，写入失败
    }

    rb->buffer[rb->tail] = byte;
    rb->tail = (rb->tail + 1) % rb->size; // 尾指针前进，并回环

    // 如果写入后，头尾指针相遇，则表示缓冲区已满
    if (rb->head == rb->tail) {
        rb->is_full = true;
        // 报错
        while(1){
            delay_cycles(CPUCLK_FREQ);
            UART_send_string(UART_BLUEUART_INST, "---RingBuffer is full---\r\n");
        }
    }

    return true;
}

// 从缓冲区读取一个字节（由主循环调用）
bool RingBuffer_Read(RingBuffer_t *rb, uint8_t *byte) {
    if (RingBuffer_IsEmpty(rb)) {
        return false; // 缓冲区为空，读取失败
    }

    *byte = rb->buffer[rb->head];
    rb->head = (rb->head + 1) % rb->size; // 头指针前进，并回环

    rb->is_full = false; // 只要进行过一次读取，缓冲区就不可能为满

    return true;
}

// 检查缓冲区是否为空
bool RingBuffer_IsEmpty(const RingBuffer_t *rb) {
    // 缓冲区为空的唯一条件是：头尾指针相同，且缓冲区不满
    return (!rb->is_full && (rb->head == rb->tail));
}