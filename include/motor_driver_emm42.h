/**
 * @file motor_driver_emm42.h
 * @brief Emm42_V5.0 闭环步进驱动串口控制接口设计（仅头文件，暂不提供实现）。
 *
 * 设计目标：
 * - 提供语义化的 Emm42 控制 API（使能、速度模式、位置模式、紧急停止、多机同步等）；
 * - 封装地址、功能码与校验等细节，对上层隐藏具体协议格式；
 * - 与现有 STEP/DIR 控制链路解耦，仅作为“驱动配置/诊断通路”。
 */

#ifndef LOWER_MOTOR_DRIVER_EMM42_H
#define LOWER_MOTOR_DRIVER_EMM42_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <stdbool.h>

/**
 * @brief Emm42 校验模式。
 */
typedef enum
{
    EMM42_CHECKSUM_FIXED_0x6B = 0U, /**< 固定 0x6B 作为校验字节（默认模式） */
    EMM42_CHECKSUM_XOR,             /**< 对前面所有字节做 XOR 校验 */
    EMM42_CHECKSUM_CRC8,            /**< 对前面所有字节做 CRC-8 校验 */
} Emm42_ChecksumMode;

/**
 * @brief Emm42 通用状态码。
 */
typedef enum
{
    EMM42_OK = 0,        /**< 操作成功 */
    EMM42_ERR_PARAM,     /**< 参数错误（指针为空、数值超界等） */
    EMM42_ERR_TIMEOUT,   /**< 等待驱动器回复超时 */
    EMM42_ERR_BUSY,      /**< 当前仍有命令在执行或串口忙 */
    EMM42_ERR_HW,        /**< 硬件错误（例如条件不满足、驱动报警） */
} Emm42_Status;

/**
 * @brief Emm42 句柄结构体。
 *
 * @note 句柄本身不管理 UART 生命周期，由外部完成 UART 初始化。
 */
typedef struct
{
    UART_HandleTypeDef *huart;   /**< 串口句柄（通常为 &huart1） */
    uint8_t             id_addr; /**< 驱动器地址 (1–255)，0 预留为广播 */
    Emm42_ChecksumMode  checksum_mode; /**< 校验模式 */
} Emm42_Handle;

/**
 * @brief 初始化 Emm42 句柄。
 *
 * @param[out] handle   句柄对象指针，不可为 NULL
 * @param[in]  huart    UART 句柄指针，例如 &huart1
 * @param[in]  id_addr  设备地址（1–255，0 预留为广播）
 * @return EMM42_OK 表示句柄初始化成功
 *
 * @note 本函数仅设置本地句柄字段，不启动 DMA 或修改 UART 配置。
 */
Emm42_Status Emm42_Init(Emm42_Handle *handle,
                        UART_HandleTypeDef *huart,
                        uint8_t id_addr);

/**
 * @brief 设置 Emm42 校验模式（0x6B / XOR / CRC8）。
 *
 * @param[in,out] handle  句柄对象指针
 * @param[in]     mode    校验模式
 * @return EMM42_OK 表示设置成功
 */
Emm42_Status Emm42_SetChecksumMode(Emm42_Handle *handle,
                                   Emm42_ChecksumMode mode);

/**
 * @brief 控制电机使能/失能。
 *
 * @param[in,out] handle  句柄对象指针
 * @param[in]     enable  true 表示使能，false 表示失能
 * @param[in]     sync    多机同步标志（对应文档中的同步控制位）
 * @return EMM42_OK 表示命令发送并得到成功状态回复
 *
 * @note 对应说明书中的“电机使能控制”命令（F3 AB）。
 */
Emm42_Status Emm42_Enable(Emm42_Handle *handle,
                          bool enable,
                          bool sync);

/**
 * @brief 速度模式控制。
 *
 * @param[in,out] handle  句柄对象指针
 * @param[in]     ccw     true 表示 CCW 方向，false 表示 CW 方向
 * @param[in]     rpm     目标转速（单位 RPM，需符合驱动器允许范围）
 * @param[in]     accel   加速度档位（0 表示不启用曲线加减速）
 * @param[in]     sync    多机同步标志
 * @return EMM42_OK 表示命令发送并得到成功状态回复
 *
 * @note 对应说明书中的“速度模式控制”命令（F6）。
 */
Emm42_Status Emm42_SetSpeed(Emm42_Handle *handle,
                            bool ccw,
                            uint16_t rpm,
                            uint8_t accel,
                            bool sync);

/**
 * @brief 位置模式控制（相对/绝对）。
 *
 * @param[in,out] handle        句柄对象指针
 * @param[in]     ccw           true 表示 CCW 方向，false 表示 CW
 * @param[in]     rpm           目标运行速度（RPM）
 * @param[in]     accel         加速度档位
 * @param[in]     pulses        目标脉冲数（细分后的脉冲数）
 * @param[in]     use_absolute  true 表示绝对位置模式，false 表示相对位置模式
 * @param[in]     sync          多机同步标志
 * @return EMM42_OK 表示命令发送并得到成功状态回复
 *
 * @note 对应说明书中的“位置模式控制”命令（FD）。
 */
Emm42_Status Emm42_MoveRelative(Emm42_Handle *handle,
                                bool ccw,
                                uint16_t rpm,
                                uint8_t accel,
                                uint32_t pulses,
                                bool use_absolute,
                                bool sync);

/**
 * @brief 立即停止（紧急刹车）。
 *
 * @param[in,out] handle  句柄对象指针
 * @param[in]     sync    多机同步标志
 * @return EMM42_OK 表示命令发送并得到成功状态回复
 *
 * @note 对应说明书中的“立即停止”命令（FE 98）。
 */
Emm42_Status Emm42_StopNow(Emm42_Handle *handle,
                           bool sync);

/**
 * @brief 多机同步启动。
 *
 * @param[in,out] handle  句柄对象指针
 * @return EMM42_OK 表示命令发送并得到成功状态回复
 *
 * @note 对应说明书中的“多机同步运动”命令（FF 66），
 *       需配合带同步标志的速度/位置命令使用。
 */
Emm42_Status Emm42_SyncStart(Emm42_Handle *handle);

#ifdef __cplusplus
}
#endif

#endif /* LOWER_MOTOR_DRIVER_EMM42_H */
