#ifndef __SEND_H
#define __SEND_H

#include <stdint.h>

// 打包数据，返回数据长度
uint8_t data_frame_made(uint8_t *out_buffer);

// 触发 DMA 发送
void send_chassis_data(uint8_t *data, uint8_t len);

#endif