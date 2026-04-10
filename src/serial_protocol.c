/**
 * @file serial_protocol.c
 * @brief 协议帧调度与命令映射实现。
 *
 * 数据流总览：
 * 1. USART2 + DMA + IDLE 在 dma.c 中将新到字节段送入 protocol_feed_stream；
 * 2. protocol_feed_stream 通过简单状态机（WAIT_SOF1/WAIT_SOF2/READ_HEADER/READ_PAYLOAD）
 *    将字节流组装成完整帧；
 * 3. 组帧完成后调用 protocol_dispatch，进行 SOF/长度/CRC 校验并按 FUNC/CMD 查表分发；
 * 4. 各 handler 在完成任务后根据 need_ack 标志由 protocol_send_ack 生成 ACK。
 *
 * 该模块处于“串口搬运（dma.c）”与“业务逻辑（chassis/JY901B/...）”之间，
 * 不直接操作定时器或底层硬件，仅负责协议约束与命令路由。
 */

#include "serial_protocol.h"
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "usart.h"
#include "dma.h"
#include "JY901B.h"
#include "chassis.h"

#define PROTOCOL_TX_TIMEOUT_MS   (5U)

#ifndef FIRMWARE_VERSION_MAJOR
#define FIRMWARE_VERSION_MAJOR  0U
#endif
#ifndef FIRMWARE_VERSION_MINOR
#define FIRMWARE_VERSION_MINOR  1U
#endif
#ifndef SYSINFO_BOARD_TYPE
#define SYSINFO_BOARD_TYPE     0x01U        /**< 设备类型：默认底盘主控 */
#endif
#ifndef SYSINFO_FEATURES
#define SYSINFO_FEATURES       0x01U        /**< 功能位：bit0=IMU，bit1=编码器，bit2=舵机等 */
#endif

_Static_assert(PROTOCOL_MAX_PAYLOAD_BYTES + FRAME_HEADER_SIZE + FRAME_CRC_SIZE <= MAX_RX_DMA_FRAME_SIZE,
               "DMA 缓冲大小必须覆盖协议最大帧长");

typedef struct __attribute__((packed))
{
    uint8_t proto_ver;   /**< 协议版本，与 CFG 高 4 位一致 */
    uint8_t fw_major;    /**< 固件主版本 */
    uint8_t fw_minor;    /**< 固件次版本 */
    uint8_t board_type;  /**< 硬件/板卡类型 */
    uint8_t features;    /**< 功能位掩码：bit0=IMU，bit1=编码器，bit2=舵机... */
} ProtocolInfoPayload;



/*--------------------------- handler 声明 ---------------------------*/
static ProtocolHandlerResult handle_sys_ping(const FrameHeader *h, const uint8_t *data, uint16_t len);
static ProtocolHandlerResult handle_sys_reboot(const FrameHeader *h, const uint8_t *data, uint16_t len);
static ProtocolHandlerResult handle_sys_get_info(const FrameHeader *h, const uint8_t *data, uint16_t len);
static ProtocolHandlerResult handle_chassis_set_vel(const FrameHeader *h, const uint8_t *data, uint16_t len);
static ProtocolHandlerResult handle_sensor_get_imu(const FrameHeader *h, const uint8_t *data, uint16_t len);
static ProtocolHandlerResult handle_chassis_set_pos(const FrameHeader *h, const uint8_t *data, uint16_t len);
static ProtocolHandlerResult handle_chassis_get_state(const FrameHeader *h, const uint8_t *data, uint16_t len);
/*--------------------------- 辅助函数 ---------------------------*/
static uint8_t protocol_compose_cfg(uint8_t flags);
static ProtocolAckCode protocol_ack_from_result(ProtocolHandlerResult result);
static void protocol_send_ack(const FrameHeader *request, ProtocolAckCode code);
static uint8_t protocol_alloc_seq(void);
static HAL_StatusTypeDef protocol_tx_frame(uint8_t func,
                                           uint8_t cmd,
                                           const void *payload,
                                           uint16_t len,
                                           uint8_t dst,
                                           uint8_t cfg_flags,
                                           uint8_t seq,
                                           bool force_seq);

/*--------------------------- 命令映射表 ---------------------------*/
static const CommandMap g_cmd_table[] = {
    {FUNC_SYS,     CMD_SYS_PING,        handle_sys_ping},
    {FUNC_SYS,     CMD_SYS_REBOOT,      handle_sys_reboot},
    {FUNC_SYS,     CMD_SYS_GET_INFO,    handle_sys_get_info},
    {FUNC_CHASSIS, CMD_CHASSIS_SET_VEL, handle_chassis_set_vel},
    {FUNC_CHASSIS, CMD_CHASSIS_SET_POS,   handle_chassis_set_pos},
    {FUNC_CHASSIS, CMD_CHASSIS_GET_STATE, handle_chassis_get_state},
    {FUNC_SENSOR,  CMD_SENSOR_GET_IMU,  handle_sensor_get_imu},
};

/**
 * @brief 查找 func/cmd 对应的处理函数
 * @param func 功能 ID
 * @param cmd 命令 ID
 * @return 匹配到的命令表项，否则 NULL
 * @note 纯软件查表，不依赖硬件资源
 */
static const CommandMap* find_command(uint8_t func, uint8_t cmd)
{
    const size_t table_size = sizeof(g_cmd_table) / sizeof(g_cmd_table[0]);
    for (size_t i = 0; i < table_size; ++i)
    {
        if (g_cmd_table[i].func == func && g_cmd_table[i].cmd == cmd)
        {
            return &g_cmd_table[i];
        }
    }
    return NULL;
}

/**
 * @brief 解析并调度完整帧。
 * @param[in] frame_buf 指向完整帧数据（从 SOF 开始）
 * @param[in] frame_len 帧总长度（包含 SOF/HEADER/DATA/CRC）
 * @note  输入由 protocol_feed_stream 保证按帧对齐，函数内部负责长度/CRC 校验与 handler 调用。
 */
void protocol_dispatch(const uint8_t *frame_buf, uint16_t frame_len)
{
    // 1. 基本长度检查（至少要包含帧头与 CRC）
    if (frame_len < FRAME_HEADER_SIZE + FRAME_CRC_SIZE) { return; }

    const FrameHeader *header = (const FrameHeader *)frame_buf;
    if ((header->cfg & FRAME_CFG_IS_ACK) != 0U)
    {
        /* 对端 ACK 仅供上位机处理，底盘侧无需回 ACK */
        return;
    }

    // 2. 起始标志校验
    if (header->sof1 != FRAME_SOF1 || header->sof2 != FRAME_SOF2) { return; }

    // 3. 数据长度合法性检查（防止越界）
    const uint16_t data_len = header->len;
    if ((FRAME_HEADER_SIZE + data_len + FRAME_CRC_SIZE) != frame_len) { return; }

    const uint8_t *data = frame_buf + FRAME_HEADER_SIZE;
    const uint8_t *crc_ptr = data + data_len;

    // 4. CRC16 校验（header + payload）
    const uint16_t crc_calc = crc16(frame_buf, (uint16_t)(FRAME_HEADER_SIZE + data_len));
    const uint16_t crc_recv = (uint16_t)crc_ptr[0] | ((uint16_t)crc_ptr[1] << 8);
    if (crc_calc != crc_recv) { return; }

    const bool need_ack = ((header->cfg & FRAME_CFG_NEED_ACK) != 0U);
    ProtocolAckCode ack_code = PROTOCOL_ACK_OK;

    // 5. func/cmd 查表并调用 handler
    const CommandMap *entry = find_command(header->func, header->cmd);
    if (!entry || !entry->handler)
    {
        ack_code = PROTOCOL_ACK_ERR_UNSUPPORTED;
    }
    else
    {
        ProtocolHandlerResult result = entry->handler(header, data, data_len);
        ack_code = protocol_ack_from_result(result);
    }

    if (need_ack)
    {
        protocol_send_ack(header, ack_code);
    }
}

/*--------------------------- handler 示例 ---------------------------*/
/**
 * @brief 底盘速度设置命令处理：解析 4 个轮速并写入 Chassis 模块。
 * @param[in] h    请求帧头指针（用于获取 seq 等，可为空）
 * @param[in] data 载荷指针，应为 4×int16 小端编码
 * @param[in] len  载荷长度，必须为 8
 * @return 处理结果码，用于映射 ACK
 */
static ProtocolHandlerResult handle_chassis_set_vel(const FrameHeader *h,
                                                    const uint8_t *data,
                                                    uint16_t len)
{
    // 1. 长度检查
    if (len != 8U)
    {
        return PROTOCOL_HANDLER_ERR_PARAM;
    }

    // 2. 按顺序解出四个 int16
    const int16_t cmd_raw[4] = {
        (int16_t)(data[0] | (data[1] << 8)),
        (int16_t)(data[2] | (data[3] << 8)),
        (int16_t)(data[4] | (data[5] << 8)),
        (int16_t)(data[6] | (data[7] << 8)),
    };

    if (!Chassis_ApplyWheelCommand(cmd_raw, h ? h->seq : 0U))
    {
        return PROTOCOL_HANDLER_ERR_INTERNAL;
    }

    return PROTOCOL_HANDLER_OK;
}

static ProtocolHandlerResult handle_sys_ping(const FrameHeader *h, const uint8_t *data, uint16_t len)
{
    (void)h;
    (void)data;
    (void)len;
    /* 心跳命令仅需等待 protocol_dispatch 回 ACK */
    return PROTOCOL_HANDLER_OK;
}

static ProtocolHandlerResult handle_sys_reboot(const FrameHeader *h, const uint8_t *data, uint16_t len)
{
    (void)h;
    (void)data;
    (void)len;
    /* 软复位功能暂未实现，返回错误状态避免上位机误判 */
    return PROTOCOL_HANDLER_ERR_UNSUPPORTED;
}

static ProtocolHandlerResult handle_sys_get_info(const FrameHeader *h,
                                                 const uint8_t *data,
                                                 uint16_t len)
{
    (void)data;
    (void)len;

    ProtocolInfoPayload payload = {
        .proto_ver = PROTOCOL_VERSION,
        .fw_major = FIRMWARE_VERSION_MAJOR,
        .fw_minor = FIRMWARE_VERSION_MINOR,
        .board_type = SYSINFO_BOARD_TYPE,
        .features = SYSINFO_FEATURES,
    };

    const HAL_StatusTypeDef status = protocol_tx_frame(FUNC_SYS,
                                                       CMD_SYS_GET_INFO,
                                                       &payload,
                                                       (uint16_t)sizeof(payload),
                                                       h ? h->src : PROTOCOL_DEFAULT_REMOTE_ADDR,
                                                       0U,
                                                       h ? h->seq : 0U,
                                                       true);
    return (status == HAL_OK) ? PROTOCOL_HANDLER_OK : PROTOCOL_HANDLER_ERR_INTERNAL;
}

static ProtocolHandlerResult handle_sensor_get_imu(const FrameHeader *h,
                                                   const uint8_t *data,
                                                   uint16_t len)
{
    (void)data;
    (void)len;

    /* 发送 32B 原始寄存器窗口（0x30~0x4F），与主动推送保持一致 */
    const HAL_StatusTypeDef status = protocol_tx_frame(FUNC_SENSOR,
                                                       CMD_SENSOR_GET_IMU,
                                                       g_jy901_raw,
                                                       (uint16_t)JY901_I2C_BLOCK_BYTES,
                                                       h->src,
                                                       0U,
                                                       h->seq,
                                                       true);
    return (status == HAL_OK) ? PROTOCOL_HANDLER_OK : PROTOCOL_HANDLER_ERR_INTERNAL;
}


/**
 * @brief 底盘位置设置命令处理：解析 4 个轮子的目标脉冲并写入 Chassis 模块。
 */
static ProtocolHandlerResult handle_chassis_set_pos(const FrameHeader *h, const uint8_t *data, uint16_t len)
{
    if (len != sizeof(Payload_ChassisSetPos)) {
        return PROTOCOL_HANDLER_ERR_PARAM;
    }
    Payload_ChassisSetPos payload;
    memcpy(&payload, data, sizeof(Payload_ChassisSetPos));
    // 提取 4 个轮子的脉冲目标值
    int32_t pulses[4] = {payload.p1, payload.p2, payload.p3, payload.p4};
    // 调用 chassis.c 里真正存在的函数！
    if (!Chassis_ApplyPositionCommand(pulses, h ? h->seq : 0U)) {
        return PROTOCOL_HANDLER_ERR_INTERNAL;
    }
    return PROTOCOL_HANDLER_OK;
}
/**
 * @brief 处理上位机下发的“获取底盘状态”指令
 */
static ProtocolHandlerResult handle_chassis_get_state(const FrameHeader *h, const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
    Payload_ChassisState state;
    memset(&state, 0, sizeof(state));
    // 调用 chassis.c 里真正存在的函数！
    ChassisSetpoint sp;
    if (Chassis_GetSetpoint(&sp)) {
        if (sp.mode == CHASSIS_MODE_SPEED) {
            state.v1 = sp.raw_cmd[0];
            state.v2 = sp.raw_cmd[1];
            state.v3 = sp.raw_cmd[2];
            state.v4 = sp.raw_cmd[3];
        } else if (sp.mode == CHASSIS_MODE_POSITION) {
            state.p1 = sp.wheel_pulses[0];
            state.p2 = sp.wheel_pulses[1];
            state.p3 = sp.wheel_pulses[2];
            state.p4 = sp.wheel_pulses[3];
        }
    }
    // 组包发回给 PC
    uint8_t cfg_flags = protocol_compose_cfg(0); 
    protocol_tx_frame(FUNC_CHASSIS, CMD_CHASSIS_GET_STATE, 
                      &state, sizeof(state), 
                      h->src, cfg_flags, h->seq, true);
    return PROTOCOL_HANDLER_OK;
}


/**
 * @brief 组合带版本号的 CFG 字段。
 * @param[in] flags 标志位（NEED_ACK/IS_ACK 等）
 * @return 带版本号编码后的 CFG 字节
 * @note  协议版本来自 PROTOCOL_VERSION，并被编码到 CFG 高 4 位。
 */
static uint8_t protocol_compose_cfg(uint8_t flags)
{
    const uint8_t ver = (uint8_t)((PROTOCOL_VERSION << FRAME_CFG_VERSION_SHIFT) & FRAME_CFG_VERSION_MASK);
    return (uint8_t)(ver | flags);
}

/**
 * @brief 分配下行帧序号，避免与 ACK 冲突。
 * @return 新分配的 SEQ 值（0~255 环绕）
 * @note  仅用于“主动下行”帧，响应帧则复用请求帧的 seq。
 */
static uint8_t protocol_alloc_seq(void)
{
    static uint8_t s_seq = 0U;
    s_seq++;
    return s_seq;
}

/**
 * @brief handler 结果到 ACK 码的映射。
 * @param[in] result handler 返回值
 * @return 对应的 ProtocolAckCode
 */
static ProtocolAckCode protocol_ack_from_result(ProtocolHandlerResult result)
{
    switch (result)
    {
    case PROTOCOL_HANDLER_OK:
        return PROTOCOL_ACK_OK;
    case PROTOCOL_HANDLER_ERR_PARAM:
        return PROTOCOL_ACK_ERR_PARAM;
    case PROTOCOL_HANDLER_ERR_UNSUPPORTED:
        return PROTOCOL_ACK_ERR_UNSUPPORTED;
    case PROTOCOL_HANDLER_ERR_INTERNAL:
    default:
        return PROTOCOL_ACK_ERR_INTERNAL;
    }
}

/**
 * @brief 根据请求帧构造最小 ACK。
 * @param[in] request 原始请求帧头指针，可为空（为空时使用默认地址/命令）
 * @param[in] code    ACK 结果码，见 ProtocolAckCode
 * @note  ACK 载荷固定 1 字节，采用阻塞发送保证在短时间内返回。
 */
static void protocol_send_ack(const FrameHeader *request, ProtocolAckCode code)
{
    uint8_t frame[FRAME_HEADER_SIZE + 1U + FRAME_CRC_SIZE];
    FrameHeader *ack = (FrameHeader *)frame;

    ack->sof1 = FRAME_SOF1;
    ack->sof2 = FRAME_SOF2;
    ack->cfg  = protocol_compose_cfg(FRAME_CFG_IS_ACK);
    ack->seq  = request ? request->seq : 0U;
    ack->src  = PROTOCOL_NODE_ADDR;
    ack->dst  = request ? request->src : PROTOCOL_DEFAULT_REMOTE_ADDR;
    ack->func = request ? request->func : 0U;
    ack->cmd  = request ? request->cmd : 0U;
    ack->len  = 1U;

    uint8_t *payload = frame + FRAME_HEADER_SIZE;
    payload[0] = (uint8_t)code;

    const uint16_t crc = crc16(frame, (uint16_t)(FRAME_HEADER_SIZE + ack->len));
    uint8_t *crc_ptr = payload + ack->len;
    crc_ptr[0] = (uint8_t)(crc & 0xFFU);
    crc_ptr[1] = (uint8_t)(crc >> 8);

    // 非阻塞入队（ISR 安全）
    (void)DMA_UART2_TxEnqueue(frame,
                             (uint16_t)(FRAME_HEADER_SIZE + ack->len + FRAME_CRC_SIZE));
}

/**
 * @brief 统一的下行帧封装与发送函数。
 * @param[in] func       功能域 ID（FUNC_SYS/FUNC_CHASSIS/FUNC_SENSOR...）
 * @param[in] cmd        命令 ID
 * @param[in] payload    载荷指针，可为空（len=0 时忽略）
 * @param[in] len        载荷长度，必须小于等于 PROTOCOL_MAX_PAYLOAD_BYTES
 * @param[in] dst        目的地址
 * @param[in] cfg_flags  CFG 标志位（NEED_ACK/IS_ACK 等），不包含版本位
 * @param[in] seq        指定 SEQ 值（force_seq 为 true 时生效）
 * @param[in] force_seq  是否强制使用传入 seq，否则自动分配
 * @return HAL_OK 表示已成功写入 DMA 发送 FIFO；其他返回值表示参数错误或超时
 */
static HAL_StatusTypeDef protocol_tx_frame(uint8_t func,
                                           uint8_t cmd,
                                           const void *payload,
                                           uint16_t len,
                                           uint8_t dst,
                                           uint8_t cfg_flags,
                                           uint8_t seq,
                                           bool force_seq)
{
    if (len > PROTOCOL_MAX_PAYLOAD_BYTES)
    {
        return HAL_ERROR;
    }

    uint8_t frame[FRAME_HEADER_SIZE + PROTOCOL_MAX_PAYLOAD_BYTES + FRAME_CRC_SIZE];
    FrameHeader *header = (FrameHeader *)frame;

    header->sof1 = FRAME_SOF1;
    header->sof2 = FRAME_SOF2;
    header->cfg  = protocol_compose_cfg(cfg_flags);
    header->seq  = force_seq ? seq : protocol_alloc_seq();
    header->src  = PROTOCOL_NODE_ADDR;
    header->dst  = dst;
    header->func = func;
    header->cmd  = cmd;
    header->len  = len;

    if (len > 0U)
    {
        if (payload == NULL)
        {
            return HAL_ERROR;
        }
        memcpy(frame + FRAME_HEADER_SIZE, payload, len);
    }

    const uint16_t crc = crc16(frame, (uint16_t)(FRAME_HEADER_SIZE + len));
    uint8_t *crc_ptr = frame + FRAME_HEADER_SIZE + len;
    crc_ptr[0] = (uint8_t)(crc & 0xFFU);
    crc_ptr[1] = (uint8_t)(crc >> 8);

    const uint16_t total = (uint16_t)(FRAME_HEADER_SIZE + len + FRAME_CRC_SIZE);

    // ISR 中：非阻塞入队；主循环：可等待 FIFO 空间
    if (__get_IPSR() != 0U)
    {
        return DMA_UART2_TxEnqueue(frame, total);
    }
    else
    {
        return DMA_UART2_TxSend(frame, total, PROTOCOL_TX_TIMEOUT_MS);
    }
}

HAL_StatusTypeDef protocol_send_sensor_payload(uint8_t cmd,
                                               const void *payload,
                                               uint16_t len)
{
    /**
     * @brief 发送传感器相关 Telemetry 帧的便捷封装。
     * @param[in] cmd     传感器命令 ID（如 CMD_SENSOR_GET_IMU）
     * @param[in] payload 载荷指针
     * @param[in] len     载荷长度
     * @note  默认发送到 PROTOCOL_DEFAULT_REMOTE_ADDR，CFG 不带 ACK 标志。
     */
    return protocol_tx_frame(FUNC_SENSOR,
                             cmd,
                             payload,
                             len,
                             PROTOCOL_DEFAULT_REMOTE_ADDR,
                             0U,
                             0U,
                             false);
}

typedef enum
{
    STATE_WAIT_SOF1,
    STATE_WAIT_SOF2,
    STATE_READ_HEADER,
    STATE_READ_PAYLOAD,
} ProtocolState;

static void protocol_reset_state(ProtocolState *state, uint16_t *frame_len, uint16_t *target_len)
{
    *state = STATE_WAIT_SOF1;
    *frame_len = 0U;
    *target_len = 0U;
}

/**
 * @brief 串口字节流状态机入口
 * @param[in] buf   新增字节缓冲（连续）
 * @param[in] count 新字节数量
 * @note  输入来源：USART2 + DMA 环形缓冲；状态机按 SOF→HEADER→PAYLOAD 流程拼帧并调用 protocol_dispatch。
 */
void protocol_feed_stream(const uint8_t *buf, uint16_t count)
{
    static ProtocolState state = STATE_WAIT_SOF1;
    static uint8_t frame_buffer[MAX_RX_DMA_FRAME_SIZE];
    static uint16_t frame_len = 0U;
    static uint16_t target_len = 0U;

    for (uint16_t i = 0U; i < count; ++i)
    {
        const uint8_t byte = buf[i];

        switch (state)
        {
        case STATE_WAIT_SOF1:
            /* 1. 等待 SOF1（0xA5），过滤掉所有无关字节 */
            if (byte == FRAME_SOF1)
            {
                frame_buffer[0] = byte;
                frame_len = 1U;
                state = STATE_WAIT_SOF2;
            }
            break;

        case STATE_WAIT_SOF2:
            /* 2. 等待 SOF2（0x5A），若不匹配则回到初始状态重新搜索帧头 */
            if (byte == FRAME_SOF2)
            {
                frame_buffer[frame_len++] = byte;
                state = STATE_READ_HEADER;
            }
            else
            {
                protocol_reset_state(&state, &frame_len, &target_len);
            }
            break;

        case STATE_READ_HEADER:
            /* 3. 按字节读取完整帧头，获取 payload 长度信息 */
            frame_buffer[frame_len++] = byte;
            if (frame_len == FRAME_HEADER_SIZE)
            {
                const FrameHeader *header = (const FrameHeader *)frame_buffer;
                target_len = (uint16_t)(FRAME_HEADER_SIZE + header->len + FRAME_CRC_SIZE);
                if (header->len > PROTOCOL_MAX_PAYLOAD_BYTES || target_len > MAX_RX_DMA_FRAME_SIZE)
                {
                    /* 长度超限，丢弃当前帧并重置状态机，防止越界 */
                    protocol_reset_state(&state, &frame_len, &target_len);
                }
                else
                {
                    state = STATE_READ_PAYLOAD;
                }
            }
            break;

        case STATE_READ_PAYLOAD:
            /* 4. 按预期长度收齐 payload + CRC，收齐后交给 protocol_dispatch 做校验与分发 */
            frame_buffer[frame_len++] = byte;
            if (frame_len == target_len)
            {
                protocol_dispatch(frame_buffer, frame_len);
                protocol_reset_state(&state, &frame_len, &target_len);
            }
            break;

        default:
            protocol_reset_state(&state, &frame_len, &target_len);
            break;
        }
    }
}

/**
 * @brief 计算 CRC-16/IBM
 * @param buf 输入缓冲（不可为 NULL）
 * @param len 字节数
 * @return CRC 结果，小端
 * @note 多项式 0x8005，初始值 0xFFFF，refin/refout = true
 */
uint16_t crc16(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0U)
    {
        return 0U;
    }

    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; ++i)
    {
        crc ^= buf[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
