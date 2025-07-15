#include "mavlink.h"
#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "user.h"


uint8_t g_target_system_id = 0; // 用来存储无人机的ID，从心跳包中获取
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
            // heartbeat
            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                g_target_system_id = msg.sysid;
                // 打印
                UART_send(0xAA);
                UART_send(heartbeat.autopilot);
                UART_send(heartbeat.base_mode);
                UART_send(heartbeat.custom_mode);
                UART_send(heartbeat.system_status);
                UART_send(heartbeat.mavlink_version);
                UART_send(0xAA);
            }
            break;
            // UWB和陀螺仪
            case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
            {
                mavlink_global_vision_position_estimate_t uwb;
                mavlink_msg_global_vision_position_estimate_decode(&msg, &uwb);
                // 这里xyz 均为float 只是为了打印测试转化为uint8
                UART_send(0xBB);
                UART_send((uint8_t)uwb.x);
                UART_send((uint8_t)uwb.y);
                UART_send((uint8_t)uwb.z);
                UART_send(0xBB);
            }
            break;
            default:
                // 对于不关心的消息，什么都不做
                break;
        }
    }
}






// /**
//  * @brief 向无人机请求（或停止）一个数据流
//  * 
//  * @param target_system   目标无人机的系统ID (通常是 1)
//  * @param target_component 目标无人机的组件ID (通常是 1)
//  * @param stream_id       要请求的数据流ID 
//  * @param rate_hz         请求的数据更新频率 (Hz)。如果为0，则停止该数据流。
//  */
// void request_data_stream(uint8_t target_system, uint8_t target_component, uint8_t stream_id, uint16_t rate_hz)
// {
//     // 转化为v1 包格式发送
//     // 1. 获取指定通信通道的状态结构体指针。
//     //    mavlink_status_t 不仅用于接收状态统计，也用于控制通道行为。
//     mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);
//     // 2. 修改 flags 字段。
//     //    使用按位或（|=）操作，将 MAVLINK_STATUS_FLAG_OUT_MAVLINK1 这个标志位置为1。
//     chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;


//     uint8_t g_my_system_id = 255;      // 您的设备（MCU）的系统ID，地面站/伴侣计算机通常用255
//     uint8_t g_my_component_id = 0;   // 您的设备（MCU）的组件ID，通常用0或MAV_COMP_ID_SYSTEM_CONTROL

//     // 1. 定义一个用于存放打包后消息的 mavlink_message_t 结构体
//     mavlink_message_t msg;

//     // 2. 定义一个缓冲区用于存放序列化后的字节流
//     uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//     // 3. 打包 REQUEST_DATA_STREAM 消息
//     //    参数依次为：我方SysID, 我方CompID, 消息存放指针, 目标SysID, 目标CompID, 流ID, 频率, 启动/停止标志
//     mavlink_msg_request_data_stream_pack(
//         g_my_system_id,      // 我们的系统ID
//         g_my_component_id,   // 我们的组件ID
//         &msg,
//         target_system,       // 无人机的系统ID
//         target_component,    // 无人机的组件ID
//         stream_id,           // 要请求的数据流ID
//         rate_hz,             // 请求的频率
//         1                    // 1 = 开始发送; 0 = 停止发送
//     );

//     // 4. 将打包好的 msg 序列化到 buf 中
//     uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

//     // 5. 通过您的底层串口发送函数，将数据发送出去
//     UART_send_buffer(UART_MAVLINK_INST, buf, len);
// }





/**
 * @brief (为V1固件优化) 请求无人机以指定频率发送某个特定的消息ID
 *        此函数会强制以 MAVLink V1 格式发送请求。
 * 
 * @param target_system   目标无人机的系统ID
 * @param message_id      要订阅的消息ID (例如 MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE)
 * @param rate_hz         请求的数据更新频率 (Hz)。
 */
void request_message_interval_v1_forced(uint8_t target_system, uint16_t message_id, uint16_t rate_hz)
{
    mavlink_message_t msg; 
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // 我们设备的ID
    uint8_t g_my_system_id = 255; 
    uint8_t g_my_component_id = 0;

    // 计算时间间隔（微秒）
    int32_t interval_us = (rate_hz > 0) ? (1000000 / rate_hz) : -1; // -1 表示恢复默认频率, 0 表示停止

    // 1. 打包 COMMAND_LONG 消息 (这一步和之前一样)
    mavlink_msg_command_long_pack(
        g_my_system_id,      
        g_my_component_id,   
        &msg,                
        target_system,       
        1,                   
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,                   
        (float)message_id,   
        (float)interval_us,  
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    );
    
    // ************* 关键修改在这里 *************
    // 在序列化之前，强制将消息标记为 MAVLink V1
    // 这会使得 mavlink_msg_to_send_buffer 在内部使用 V1 的打包规则
    msg.magic = MAVLINK_STX_MAVLINK1;

    // 2. 将打包好的 COMMAND_LONG 消息序列化并发送出去
    //    因为 msg.magic 已经被设置为 V1 的起始符，所以生成的字节流将是 V1 格式。
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    UART_send_buffer(UART_MAVLINK_INST, buf, len);
}