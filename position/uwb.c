#include "mavlink.h"
#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "user.h"
#include "position.h"
#include "uwb.h"
// UWB 坐标结构体
mavlink_global_vision_position_estimate_t uwb;

/**
 * @brief 解析mavlink 串口存储在缓冲区的一个字节，在刚好解析完成一个包时可以选择触发某些操作
 * @return 无
 */
void mavlink_decode_receive_message(uint8_t byte){
    int                 chan;
    mavlink_status_t    status;
    mavlink_message_t   msg;

    chan = MAVLINK_COMM_0;
    // 处理读到的byte
    if (mavlink_parse_char(chan, byte, &msg, &status)){
        switch(msg.msgid) {
            // // heartbeat
            // case MAVLINK_MSG_ID_HEARTBEAT:{
            // }
            // break;
            // UWB和陀螺仪
            case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
            {
                // 获取UWB 坐标
                mavlink_msg_global_vision_position_estimate_decode(&msg, &uwb);             
            }
            break;
            default:
                // 对于不关心的消息，什么都不做
                break;
        }
    }
}
