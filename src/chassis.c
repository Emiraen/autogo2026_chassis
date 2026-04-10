#include "chassis.h"
#include <string.h>

/**
 * @brief 协议单位转换：每个 LSB 对应的角速度 (rad/s)。
 * @note  默认 0.01 rad/s，可根据上位机协议在 CubeMX 常量或编译选项中覆盖。
 */
#ifndef CHASSIS_CMD_LSB_RAD_PER_SEC
#define CHASSIS_CMD_LSB_RAD_PER_SEC  (0.01f)
#endif

/**
 * @brief 钳位后的最大角速度，保护驱动器。
 */
#ifndef CHASSIS_MAX_RAD_PER_SEC
#define CHASSIS_MAX_RAD_PER_SEC      (200.0f)
#endif

static ChassisSetpoint s_setpoint; /**< 保存最近一次有效的底盘指令快照（含原始值与单位换算结果） */

/**
 * @brief 角速度钳位函数，限制在 ±CHASSIS_MAX_RAD_PER_SEC 以内。
 *
 * 1. 对大于上限的值直接截断为 CHASSIS_MAX_RAD_PER_SEC；
 * 2. 对小于下限的值截断为 -CHASSIS_MAX_RAD_PER_SEC；
 * 3. 中间值原样返回。
 *
 * @param[in] value 输入角速度（rad/s）
 * @return 钳位后的角速度（rad/s）
 */
static float clamp_speed(float value)
{
    if (value > CHASSIS_MAX_RAD_PER_SEC)
    {
        return CHASSIS_MAX_RAD_PER_SEC;
    }
    if (value < -CHASSIS_MAX_RAD_PER_SEC)
    {
        return -CHASSIS_MAX_RAD_PER_SEC;
    }
    return value;
}

void Chassis_Init(void)
{
    memset(&s_setpoint, 0, sizeof(s_setpoint));
    s_setpoint.valid = false;
}

/**
 * @brief 应用新的四轮速度命令，并完成单位转换与限幅。
 *
 * 1. 从 raw[4] 中读取原始 int16 协议值；
 * 2. 通过 CHASSIS_CMD_LSB_RAD_PER_SEC 将每个轮子的值转换为 rad/s；
 * 3. 使用 clamp_speed 对每个轮子的角速度做安全钳位；
 * 4. 使用关中断的方式一次性更新全局 s_setpoint，避免与 1kHz 控制任务竞争。
 *
 * @param[in] raw 协议中携带的 4×int16（小端）数组
 * @param[in] seq 对应的协议帧序号（用于调试）
 * @return true 表示更新成功，false 表示输入指针为空
 *
 * @note 本函数通常由串口协议 handler 在 DMA/IDLE 解析后调用，内部不做阻塞操作。
 */
bool Chassis_ApplyWheelCommand(const int16_t raw[4], uint32_t seq)
{
    if (raw == NULL)
    {
        return false;
    }

    ChassisSetpoint next = s_setpoint;
    next.mode = CHASSIS_MODE_SPEED; 
    next.seq = seq;
    next.last_update_ms = HAL_GetTick();
    next.valid = true;

    for (size_t i = 0U; i < 4U; ++i)
    {
        next.raw_cmd[i] = raw[i];
        const float scaled = (float)raw[i] * CHASSIS_CMD_LSB_RAD_PER_SEC;
        next.wheel_rad_s[i] = clamp_speed(scaled);
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_setpoint = next;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return true;
}

/**
 * @brief 获取当前最新有效底盘指令的副本。
 *
 * 1. 若尚未接收到任何有效命令（valid=false），直接返回 false；
 * 2. 使用关中断方式拷贝 s_setpoint 到 out，保证数据一致性；
 * 3. 调用方可在控制循环中使用 wheel_rad_s[] 进行 STEP/DIR 映射。
 *
 * @param[out] out 输出缓冲
 * @return true 表示已有有效命令并成功拷贝，false 表示无有效命令或参数为空。
 */
bool Chassis_GetSetpoint(ChassisSetpoint *out)
{
    if (out == NULL || !s_setpoint.valid)
    {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = s_setpoint;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return true;
}
/**
 * @brief 接收上位机下发的位置指令（脉冲数或角度转换后的值）
 */
bool Chassis_ApplyPositionCommand(const int32_t pulses[4], uint32_t seq)
{
    if (pulses == NULL) return false;

    ChassisSetpoint next = s_setpoint;
    next.mode = CHASSIS_MODE_POSITION; // 标记为位置模式
    next.seq = seq;
    next.last_update_ms = HAL_GetTick();
    next.valid = true;

    for (size_t i = 0U; i < 4U; ++i) {
        next.wheel_pulses[i] = pulses[i];
        // 注意：上位机应确保脉冲数与 EMM42 细分设置一致
        // 例如：16细分时，1圈=3200脉冲
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_setpoint = next;
    if (primask == 0U) __enable_irq();

    return true;
}

