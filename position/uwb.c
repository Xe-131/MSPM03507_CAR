#include "mavlink.h"
#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "user.h"
#include "position.h"

mavlink_global_vision_position_estimate_t uwb;

/**
 * @brief 解析mavlink 串口存储在缓冲区的一个字节，在刚好解析完成一个包时可以选择触发某些操作
 * @return 无
 */
void mavlink_decode_receive_message(){
    // 缓冲区内数据已经读完
    if(uart_rx_tail == uart_rx_head){
        return;
    }

    mavlink_status_t    status;
    mavlink_message_t   msg;
    int                 chan;
    uint8_t             byte;

    chan = MAVLINK_COMM_0;
    // 读取缓冲区的数据
    byte = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
    // 处理读到的byte
    if (mavlink_parse_char(chan, byte, &msg, &status)){
        switch(msg.msgid) {
            // // heartbeat
            // case MAVLINK_MSG_ID_HEARTBEAT:
            // {
            //     mavlink_heartbeat_t heartbeat;
            //     mavlink_msg_heartbeat_decode(&msg, &heartbeat);
            //     // 打印
            //     UART_send(0xAA);
            //     UART_send(heartbeat.autopilot);
            //     UART_send(heartbeat.base_mode);
            //     UART_send(heartbeat.custom_mode);
            //     UART_send(heartbeat.system_status);
            //     UART_send(heartbeat.mavlink_version);
            //     UART_send(0xAA);
            // }
            // break;
            // UWB和陀螺仪
            case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
            {
                // mavlink_global_vision_position_estimate_t uwb;
                mavlink_msg_global_vision_position_estimate_decode(&msg, &uwb);             
            }
            break;
            default:
                // 对于不关心的消息，什么都不做
                break;
        }
    }
}
