#include "send.h"
#include "serial_protocol.h"
#include "JY901B.h"
#include "dma.h"

#define SOF1 0xA5
#define SOF2 0x5A

uint8_t data_frame_made(uint8_t *out_buffer)
{
    static uint8_t seq = 0;
    uint8_t frame_len = 0;
    uint16_t payload_len = JY901_I2C_BLOCK_BYTES; 

    out_buffer[frame_len++] = SOF1;
    out_buffer[frame_len++] = SOF2;
    out_buffer[frame_len++] = (1 << 4) | 0x00; 
    out_buffer[frame_len++] = seq++;
    out_buffer[frame_len++] = 0x10; 
    out_buffer[frame_len++] = 0x01; 
    out_buffer[frame_len++] = 0x03; // FUNC
    out_buffer[frame_len++] = 0x01; // CMD
    
    out_buffer[frame_len++] = payload_len & 0xFF;         
    out_buffer[frame_len++] = (payload_len >> 8) & 0xFF;  
    
    // 拷贝 IMU 数据
    memcpy(&out_buffer[frame_len], g_jy901_raw, payload_len);
    frame_len += payload_len;
    
    // 计算并追加 CRC
    uint16_t crc = crc16(out_buffer, frame_len);
    out_buffer[frame_len++] = crc & 0xFF;         
    out_buffer[frame_len++] = (crc >> 8) & 0xFF;  
    
    return frame_len;
}

void send_chassis_data(uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0) return;
    
    // 调用你在 dma.c 中写好的非阻塞入队函数
    DMA_UART2_TxEnqueue(data, len);
}