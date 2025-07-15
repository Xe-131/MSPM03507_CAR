#ifndef __UWB_H__
#define __UWB_H__

extern uint8_t g_target_system_id; // 用来存储无人机的ID，从心跳包中获取

void mavlink_decode_receive_message();
// void request_data_stream(uint8_t target_system, uint8_t target_component, uint8_t stream_id, uint16_t rate_hz);
void request_message_interval_v1_forced(uint8_t target_system, uint16_t message_id, uint16_t rate_hz);
#endif