#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "crcLib.h"
#include "ti_msp_dl_config.h"
#include "user.h"
#include "xe_protocol.h"
#include <string.h> 

/*
协议发送使用：
    1.针对你要发送的消息类型直接调用其对应的发送函数，即可将消息发送到对应串口

协议接收使用：
    1.在主循环中，将函数Protocol_ParseByte() 放到你想要的接收信息的串口缓冲区判断即可，比如：
        // 处理串口缓冲区数据
        if (RingBuffer_Read(&uart_pc_rx_buffer, &byte_from_buffer)) {
            // 解析树莓派指令，私有协议
            Protocol_ParseByte(byte_from_buffer);
        }
    2.该函数解析完相关指令，会根据对应的消息ID 调用其处理函数，用户可以到下方自定以处理函数

注意：
    1.目前协议接受部分的代码只支持单串口，多个串口同时接收还没实现，只有一个静态状态机变量
    2.本文件的大部分函数都调用了调试串口，写死了，串口为：UART_BLUEUART_INST
*/

/* =============================================================================
                                  接收相关实现
=============================================================================*/ 

// ---------- 内部辅助函数 ----------

// 帧处理分发函数
// 当CRC校验成功后，此函数被调用，根据数据类型将数据分发给正确的处理函数
void ProcessValidFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
    switch (type) {
        case DRONE_POSITION:
            HandleDronePosition(payload, len);
            break;
        case FIRE_SOURCE_POSITION:
            HandleFireSourcePosition(payload, len);
            break;
        // 在这里为新的数据类型添加 case
        default:
            HandleUnknownType(type);
            break;
    }
}

// 不同数据类型的处理函数
// 注意：payload 指针指向的是数据体(Data Body)的起始位置
void HandleDronePosition(const uint8_t* payload, uint8_t len) {
    if (len == 12) {
        // 在这里，你可以安全地将 payload 转换为 float 结构体指针
        float x;
        float y;
        float z;
        memcpy(&x, &payload[0], sizeof(float));
        memcpy(&y, &payload[4], sizeof(float));
        memcpy(&z, &payload[8], sizeof(float));
        UART_send_float(UART_BLUEUART_INST, x);
        UART_send_float(UART_BLUEUART_INST, y);
        UART_send_float(UART_BLUEUART_INST, z);
    } else {
        UART_send_string(UART_BLUEUART_INST, "\r\nHandler: Received DRONE_POSITION with wrong length!\r\n");
    }
}

void HandleFireSourcePosition(const uint8_t* payload, uint8_t len) {
        UART_send_string(UART_BLUEUART_INST, "\r\nFIRE_SOURCE_POSITION\r\n");

}

void HandleUnknownType(uint8_t type) {
    UART_send_string(UART_BLUEUART_INST, "\r\nXE:UNKNOW MESSAGETYPE !\r\n");
}

// ---------- 公共接口函数 ----------

// 核心协议解析函数
/**
 * @brief 协议解析状态机函数
 * @param byte 从串口接收到的单个字节
 *
 * @note 此函数不是线程安全的，因为它依赖于静态变量来维持状态。
 *       请确保只在单个任务/线程中调用它。
 */
void Protocol_ParseByte(uint8_t byte) {
    // 状态机和数据帧的静态变量，用于在多次调用间保持状态
    static ParserState_e currentState = STATE_WAIT_HEADER_1;
    static uint8_t frame_buffer[FRAME_BUFFER_SIZE];
    static uint8_t received_type;
    static uint8_t paylen_length;
    static uint16_t buffer_index;

    switch (currentState) {
        case STATE_WAIT_HEADER_1:
            if (byte == FRAME_HEADER_1) {
                currentState = STATE_WAIT_HEADER_2;
            }
            break;

        case STATE_WAIT_HEADER_2:
            if (byte == FRAME_HEADER_2) {
                buffer_index = 0; // 准备开始填充缓冲区
                currentState = STATE_READ_TYPE;
            } else {
                // 如果第二个字节不是预期的，回退到初始状态
                // 也许这个字节就是下一个包的0xAB呢？所以要重新判断一下
                currentState = (byte == FRAME_HEADER_1) ? STATE_WAIT_HEADER_2 : STATE_WAIT_HEADER_1;
            }
            break;

        case STATE_READ_TYPE:
            received_type = byte;
            frame_buffer[buffer_index++] = byte;
            currentState = STATE_READ_LENGTH;
            break;

        case STATE_READ_LENGTH:
            paylen_length = byte;
            // 安全检查：防止过大的长度导致缓冲区溢出
            if (paylen_length > MAX_PAYLOAD_SIZE) {
                UART_send_string(UART_BLUEUART_INST, "\r\nError: Frame payload length is to big\r\n");
                currentState = STATE_WAIT_HEADER_1; // 重置状态机
            } else {
                frame_buffer[buffer_index++] = byte;
                if (paylen_length == 0) {
                    // 如果没有数据体，直接去读CRC
                    currentState = STATE_READ_CRC;
                } else {
                    currentState = STATE_READ_PAYLOAD;
                }
            }
            break;

        case STATE_READ_PAYLOAD:
            frame_buffer[buffer_index++] = byte;
            // 检查是否已接收完所有数据体字节
            // 缓冲区的前2个字节是类型和长度
            if (buffer_index == (paylen_length + 2)) {
                currentState = STATE_READ_CRC;
            }
            break;

        case STATE_READ_CRC:
        {
            uint8_t received_crc = byte;
            // 对[类型, 长度, 数据体]进行CRC计算
            uint8_t calculated_crc = crc8_maxim(frame_buffer, paylen_length + 2);

            if (calculated_crc == received_crc) {
                // CRC校验成功！
                // 调用分发函数，注意数据体 payload 在缓冲区的偏移是2
                ProcessValidFrame(received_type, &frame_buffer[2], paylen_length);
            } else {
                // CRC校验失败，丢弃数据包
                // UART_send_string(UART_BLUEUART_INST, "\r\nError: CRC mismatch!\r\n");
            }
            // 无论成功还是失败，一帧处理完毕，重置状态机以接收下一帧
            currentState = STATE_WAIT_HEADER_1;
            break;
        }
    }
}




/* =============================================================================
                                  发送相关实现
=============================================================================*/ 

// ---------- 内部辅助函数 ----------

// 底层字节发送函数
void Platform_SendByte(UART_Regs *uart, uint8_t byte) {
    UART_send_byte(uart, byte);
}

// 核心协议发送函数
/**
 * @brief 打包并发送一个完整的协议帧
 * @param type     数据类型 (DataType_e)
 * @param payload  指向数据体的指针。如果无数据体，可为 NULL。
 * @param length   数据体的长度。如果无数据体，应为 0。
 */
void Protocol_SendFrame(UART_Regs *uart, uint8_t type, const uint8_t* payload, uint8_t length) {
    // 安全检查
    if (length > MAX_PAYLOAD_SIZE) {
        UART_send_string(UART_BLUEUART_INST, "\r\nError: Frame payload length is to big\r\n");
        return;
    }
    
    // 1. 发送帧头
    Platform_SendByte(uart, FRAME_HEADER_1);
    Platform_SendByte(uart, FRAME_HEADER_2);

    // 2. 发送数据类型和长度
    Platform_SendByte(uart, type);
    Platform_SendByte(uart, length);

    // 3. 发送数据体 (如果有)
    for (uint8_t i = 0; i < length; i++) {
        Platform_SendByte(uart, payload[i]);
    }
    
    // 4. 计算并发送 CRC
    // CRC 的计算范围是 [类型, 长度, 数据体]
    // 为了计算CRC，我们需要将这些部分组合在一起
    uint8_t crc_buffer[2 + MAX_PAYLOAD_SIZE]; // 创建一个足够大的临时缓冲区
    crc_buffer[0] = type;
    crc_buffer[1] = length;
    if (length > 0) {
        memcpy(&crc_buffer[2], payload, length);
    }
    
    uint8_t crc = crc8_maxim(crc_buffer, 2 + length);
    Platform_SendByte(uart, crc);
}


// ---------- 公共接口函数 ----------
/**
 * @brief 发送无人机坐标信息
 * @param x X轴坐标
 * @param y Y轴坐标
 * @param z Z轴坐标
 */
void Send_DronePosition(UART_Regs *uart, float x, float y, float z) {
    // 准备数据体：将三个 float 合并到一个字节数组中
    // C语言保证 float 是32位 (4字节)
    uint8_t payload[12]; 
    
    // 使用 memcpy 是最安全、最可移植的转换方式，可以避免对齐问题
    memcpy(&payload[0], &x, sizeof(float));
    memcpy(&payload[4], &y, sizeof(float));
    memcpy(&payload[8], &z, sizeof(float));
    
    // 调用核心发送函数
    Protocol_SendFrame(uart, DRONE_POSITION, payload, sizeof(payload));
}

// /**
//  * @brief 发送火源坐标信息 (示例)
//  * @param id   火源ID
//  * @param temp 火源温度
//  */
// void Send_FireSourcePosition(uint16_t id, float temp) {
//     // 假设火源协议的数据体是：ID(2字节) + 温度(4字节) = 6字节
//     uint8_t payload[6];
    
//     memcpy(&payload[0], &id, sizeof(uint16_t));
//     memcpy(&payload[2], &temp, sizeof(float));
    
//     Protocol_SendFrame(FIRE_SOURCE_POSITION, payload, sizeof(payload));
// }





