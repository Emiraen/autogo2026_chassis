/**
 * @file serial_protocol.h
 * @brief 帧格式与命令映射的公共定义，供上下位机共享。
 */

#ifndef LOWER_SERIAL_PROTOCOL_H
#define LOWER_SERIAL_PROTOCOL_H

#include "main.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------- 帧常量 ----------------------------------*/
#define FRAME_SOF1          (0xA5U)
#define FRAME_SOF2          (0x5AU)
#define FRAME_CRC_SIZE      (2U)

/* CFG 位定义 */
#define FRAME_CFG_NEED_ACK  (0x01U)
#define FRAME_CFG_IS_ACK    (0x02U)
#define FRAME_CFG_VERSION_SHIFT (4U)
#define FRAME_CFG_VERSION_MASK  (0xF0U)

/*--------------------------------- 节点默认配置 ----------------------------------*/
#ifndef PROTOCOL_VERSION
#define PROTOCOL_VERSION        (0x01U)  /**< 协议版本号，写入 CFG 高 4 位 */
#endif

#ifndef PROTOCOL_NODE_ADDR
#define PROTOCOL_NODE_ADDR      (0x10U)  /**< 本节点（STM32）默认地址 */
#endif

#ifndef PROTOCOL_DEFAULT_REMOTE_ADDR
#define PROTOCOL_DEFAULT_REMOTE_ADDR (0x01U) /**< 默认远端（PC）地址 */
#endif

/*--------------------------------- 帧结构 ----------------------------------*/
#pragma pack(push, 1)
/**
 * @brief 串口协议帧头，所有字段均为小端编码。
 */
typedef struct {
    uint8_t sof1;   /**< 起始字节 1，固定 0xA5 */
    uint8_t sof2;   /**< 起始字节 2，固定 0x5A */
    uint8_t cfg;    /**< 配置位：bit0 NEED_ACK，bit1 IS_ACK，bit4..7 协议版本 */
    uint8_t seq;    /**< 帧序号 0~255 */
    uint8_t src;    /**< 源地址 */
    uint8_t dst;    /**< 目的地址 */
    uint8_t func;   /**< 功能 ID（系统/底盘/舵机/传感器等） */
    uint8_t cmd;    /**< 命令 ID（功能域下的子命令） */
    uint16_t len;   /**< 数据区长度 */
} FrameHeader;

/**
 * @brief 设置底盘位置指令的 Payload (CMD_CHASSIS_SET_POS)
 */
typedef struct {
    int32_t p1; /**< 轮 1 目标位置 (脉冲) */
    int32_t p2; /**< 轮 2 目标位置 (脉冲) */
    int32_t p3; /**< 轮 3 目标位置 (脉冲) */
    int32_t p4; /**< 轮 4 目标位置 (脉冲) */
} Payload_ChassisSetPos;
/**
 * @brief 获取底盘状态响应的 Payload (CMD_CHASSIS_GET_STATE)
 */
typedef struct {
    int16_t v1, v2, v3, v4; /**< 4个轮子的当前速度 */
    int32_t p1, p2, p3, p4; /**< 4个轮子的当前位置脉冲 */
} Payload_ChassisState;

#pragma pack(pop)

#define FRAME_HEADER_SIZE   (sizeof(FrameHeader))

/*--------------------------------- 帧尺寸限制 --------------------------------*/
#define PROTOCOL_MAX_FRAME_BYTES    (1024U)                                   /**< 单帧最大字节数 */
#define PROTOCOL_HALF_FRAME_BYTES   (PROTOCOL_MAX_FRAME_BYTES / 2U)           /**< 半缓冲大小 */
#define PROTOCOL_MAX_PAYLOAD_BYTES  (PROTOCOL_MAX_FRAME_BYTES - FRAME_HEADER_SIZE - FRAME_CRC_SIZE)
#define MAX_RX_DMA_FRAME_SIZE       PROTOCOL_MAX_FRAME_BYTES // 协议允许的最大数据区长度

/**
 * @brief 处理结果枚举，对应 ACK 载荷含义。
 */
typedef enum
{
    PROTOCOL_HANDLER_OK = 0,           /**< 处理成功 */
    PROTOCOL_HANDLER_ERR_PARAM = 1,    /**< 参数或长度错误 */
    PROTOCOL_HANDLER_ERR_UNSUPPORTED,  /**< 功能/命令未实现 */
    PROTOCOL_HANDLER_ERR_INTERNAL,     /**< 其他内部错误 */
} ProtocolHandlerResult;

/**
 * @brief ACK 结果码（1 字节），与 PC 侧一致。
 */
typedef enum
{
    PROTOCOL_ACK_OK = 0x00U,           /**< 成功 */
    PROTOCOL_ACK_ERR_UNSUPPORTED = 0x01U, /**< 未实现 */
    PROTOCOL_ACK_ERR_PARAM = 0x02U,    /**< 参数错误 */
    PROTOCOL_ACK_ERR_INTERNAL = 0x03U, /**< 内部错误 */
} ProtocolAckCode;

/**
 * @brief 命令处理函数原型。
 */
typedef ProtocolHandlerResult (*CommandHandler)(const FrameHeader *h,
                                                const uint8_t *data,
                                                uint16_t len);

/**
 * @brief 功能+命令到处理函数的映射项。
 */
typedef struct
{
    uint8_t func;          /**< 功能 ID */
    uint8_t cmd;           /**< 命令 ID */
    CommandHandler handler;/**< 对应处理函数 */
} CommandMap;

/*--------------------------------- ID 枚举 ----------------------------------*/
/**
 * @brief 功能域 ID
 */
typedef enum
{
    FUNC_SYS     = 0x01U,    /**< 系统 / 调试 */
    FUNC_CHASSIS = 0x02U,    /**< 底盘 */
    FUNC_SERVO   = 0x03U,    /**< 舵机/机械臂 */
    FUNC_SENSOR  = 0x04U,    /**< 传感器 */
} ProtocolFuncId;

/**
 * @brief 系统命令 ID
 */
typedef enum
{
    CMD_SYS_PING      = 0x01U, /**< 心跳包 */
    CMD_SYS_REBOOT    = 0x02U, /**< 重启设备 */
    CMD_SYS_GET_INFO  = 0x03U, /**< 获取设备信息 */
} ProtocolSysCmd;

/**
 * @brief 底盘命令 ID
 */
typedef enum
{
    CMD_CHASSIS_SET_VEL   = 0x01U, /**< 设置底盘速度 */
    CMD_CHASSIS_SET_POS   = 0x02U, /**< 设置底盘姿态/位置 */
    CMD_CHASSIS_GET_STATE = 0x03U, /**< 请求状态 */
} ProtocolChassisCmd;

/**
 * @brief 传感器命令 ID
 */
typedef enum
{
    CMD_SENSOR_GET_IMU = 0x01U, /**< 请求 IMU 数据 */
} ProtocolSensorCmd;

/*--------------------------------- API ----------------------------------*/
uint16_t crc16(const uint8_t *buf, uint16_t len); /* 由底层 CRC 单元实现 */
void protocol_dispatch(const uint8_t *frame_buf, uint16_t frame_len);


void protocol_feed_stream(const uint8_t *buf, uint16_t count);

/**
 * @brief 通过串口协议主动发送传感器数据帧（FUNC_SENSOR）。
 * @param[in] cmd     传感器命令 ID（如 CMD_SENSOR_GET_IMU）
 * @param[in] payload 载荷指针
 * @param[in] len     载荷长度
 * @return HAL status
 */
HAL_StatusTypeDef protocol_send_sensor_payload(uint8_t cmd,
                                               const void *payload,
                                               uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* LOWER_SERIAL_PROTOCOL_H */
