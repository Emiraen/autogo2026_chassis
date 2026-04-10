/**
 * @file chassis.h
 * @brief 底盘速度命令缓存与限幅处理。
 */

#ifndef LOWER_CHASSIS_H
#define LOWER_CHASSIS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 底盘控制模式枚举
 */
typedef enum {
    CHASSIS_MODE_SPEED = 0,   /**< 速度控制模式 */
    CHASSIS_MODE_POSITION =1    /**< 位置控制模式 */
} ChassisControlMode;

/**
 * @brief 底盘指令快照。
 */
typedef struct
{
    ChassisControlMode mode;   /**< 当前控制模式 */
    float    wheel_rad_s[4];   /**< 4 个轮子的目标角速度，单位 rad/s (速度模式用) */
    int32_t  wheel_pulses[4];  /**< 4 个轮子的目标脉冲数 (位置模式用) */
    int16_t  raw_cmd[4];       /**< 原始协议数据，便于诊断 (补回这个变量) */
    
    uint32_t last_update_ms;   /**< HAL_GetTick() 时间戳 */
    uint32_t seq;              /**< 对应的协议序号 */
    bool     valid;            /**< 是否已经接收到至少一次命令 */
} ChassisSetpoint;

// --- 在底部追加一个新的函数声明 ---
/**
 * @brief 应用新的四轮位置/步数命令。
 */
bool Chassis_ApplyPositionCommand(const int32_t pulses[4], uint32_t seq);


/**
 * @brief 初始化底盘命令缓存。
 */
void Chassis_Init(void);

/**
 * @brief 应用新的四轮速度命令。
 * @param[in] raw  协议中携带的 4×int16（小端）数组
 * @param[in] seq  协议帧序号，便于调试/追踪
 * @return true 表示更新成功，false 表示参数无效
 */
bool Chassis_ApplyWheelCommand(const int16_t raw[4], uint32_t seq);

/**
 * @brief 获取当前最新指令的副本。
 * @param[out] out  输出缓冲
 * @return true 表示已有有效命令
 */
bool Chassis_GetSetpoint(ChassisSetpoint *out);

#ifdef __cplusplus
}
#endif

#endif /* LOWER_CHASSIS_H */
