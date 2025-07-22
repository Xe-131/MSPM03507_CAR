#ifndef __UWB_H__
#define __UWB_H__

#include "mavlink.h"

extern mavlink_global_vision_position_estimate_t uwb;

void mavlink_decode_receive_message(uint8_t byte);

#endif