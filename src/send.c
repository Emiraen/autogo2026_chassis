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
    
    // 我们只传 9 轴数据（加速度、角速度、欧拉角），共 24 字节
    // 或者按你现在的做法传全部 32 字节原始数据
    uint16_t payload_len = JY901_I2C_BLOCK_BYTES; 

    // 1. 帧头
    out_buffer[frame_len++] = 0xA5; // SOF1
    out_buffer[frame_len++] = 0x5A; // SOF2
    
    // 2. CFG: 版本1, 不需ACK (0x10)
    out_buffer[frame_len++] = 0x10; 
    
    // 3. SEQ / SRC / DST
    out_buffer[frame_len++] = seq++;
    out_buffer[frame_len++] = 0x10; // SRC: 下位机
    out_buffer[frame_len++] = 0x01; // DST: 上位机
    
    // 4. FUNC & CMD (严格匹配文档 3.3 节)
    out_buffer[frame_len++] = 0x04; // FUNC: 传感器 (SENSOR)
    out_buffer[frame_len++] = 0x01; // CMD: 获取 IMU (GET_IMU)
    
    // 5. LEN (小端序)
    out_buffer[frame_len++] = (uint8_t)(payload_len & 0xFF);         
    out_buffer[frame_len++] = (uint8_t)((payload_len >> 8) & 0xFF);  
    
    // 6. PAYLOAD (直接拷贝 32 字节原始数据)
    memcpy(&out_buffer[frame_len], g_jy901_raw, payload_len);
    frame_len += payload_len;
    
    // 7. CRC16 (计算范围: SOF1 到 Payload 结束)
    // 注意：确保你的 crc16 函数实现的是 CRC-16/IBM (Modbus)
    uint16_t crc = crc16(out_buffer, frame_len);
    out_buffer[frame_len++] = (uint8_t)(crc & 0xFF);         
    out_buffer[frame_len++] = (uint8_t)((crc >> 8) & 0xFF);  
    
    return frame_len;
}

void send_chassis_data(uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0) return;
    
    // 调用你在 dma.c 中写好的非阻塞入队函数
    DMA_UART2_TxEnqueue(data, len);
}
