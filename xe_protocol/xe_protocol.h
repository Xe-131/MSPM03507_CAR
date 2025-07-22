#ifndef _XE_PROTOCOL_H_
#define _XE_PROTOCOL_H_

// 协议常量定义
#define FRAME_HEADER_1      0xAB
#define FRAME_HEADER_2      0xCD
#define MAX_PAYLOAD_SIZE    255 // 数据体的最大长度
#define FRAME_BUFFER_SIZE   (1 + 1 + MAX_PAYLOAD_SIZE) // 用于CRC计算的缓冲区大小 [类型 + 长度 + 数据体]


// 协议数据类型定义
typedef enum {
    DRONE_POSITION = 0x01,
    FIRE_SOURCE_POSITION = 0x02,
    // 在这里添加更多的数据类型...
} DataType_e;

// 解析器状态机定义
typedef enum {
    STATE_WAIT_HEADER_1,
    STATE_WAIT_HEADER_2,
    STATE_READ_TYPE,
    STATE_READ_LENGTH,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC
} ParserState_e;


void ProcessValidFrame(uint8_t type, const uint8_t* payload, uint8_t len);
void HandleDronePosition(const uint8_t* payload, uint8_t len);
void HandleFireSourcePosition(const uint8_t* payload, uint8_t len);
void HandleUnknownType(uint8_t type);
void Protocol_ParseByte(uint8_t byte);

void Platform_SendByte(UART_Regs *uart, uint8_t byte);
void Protocol_SendFrame(UART_Regs *uart, uint8_t type, const uint8_t* payload, uint8_t length);
void Send_DronePosition(UART_Regs *uart, float x, float y, float z);
#endif  