/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdlib.h>  
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "JY901B.h"
#include "serial_protocol.h"
#include "chassis.h"
#include <math.h>
#include <stdio.h>
#include <sys/types.h>
#include "motor_driver_emm42.h"
#include "oled.h"
#include "send.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief PWM 输出资源描述（TIM + Channel + DIR GPIO）。
 */
typedef struct
{
  TIM_HandleTypeDef *htim;     /**< PWM 定时器句柄 */
  uint32_t           channel;  /**< PWM 通道（TIM_CHANNEL_x） */
  GPIO_TypeDef      *dir_port; /**< DIR 引脚端口 */
  uint16_t           dir_pin;  /**< DIR 引脚编号 */
} MotorPwmChannel;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TICK_1KHZ_HZ        (1000U)
#define IMU_TASK_DIV        (5U)    /* 1kHz / 5 = 200Hz */
#define CTRL_TELE_TASK_DIV  (8U)    /* 1kHz / 8 = 125Hz */
#define OLED_TASK_DIV       (250U)  /* 1kHz / 250 = 4Hz (已优化，从 50U/20Hz 降低以释放 CPU 资源) */
#define HEALTH_TASK_DIV     (1000U) /* 1kHz / 1000 = 1Hz */

/* 底盘 PWM 相关常量（TIM2/4 均由 PSC=71 → 1MHz 基准驱动） */
#define PWM_TIMER_CLK_HZ    (1000000UL) /* 72MHz / (PSC+1) */
#define PWM_MIN_FREQ_HZ     (1.0f)      /* 低于此频率直接置 0 停止输出 */
#define PWM_MAX_FREQ_HZ     (20000.0f)  /* 保护上限，避免 ARR 过小 */
#define PWM_DUTY_RATIO      (0.5f)      /* STEP 引脚保持 50% 占空比 */
#define CHASSIS_CMD_TIMEOUT_MS   (200U)   // 指令超时保护

#ifndef CHASSIS_STEPS_PER_REV
#define CHASSIS_STEPS_PER_REV   (3200.0f)          /* 细分后每圈脉冲 */
#endif
#ifndef CHASSIS_PULSE_PER_RAD
#define CHASSIS_PULSE_PER_RAD   (CHASSIS_STEPS_PER_REV / (2.0f * 3.14159265358979323846f))
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Emm42_Handle g_emm42_motors[4]; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void App_Loop(void);
void Task_Imu_200Hz(void);
void Task_ControlAndTelemetry_125Hz(void);
void Task_OLED_20Hz(void);
void Task_Health_1Hz(void);
static void ControlLoop_Apply(void);
/* USER CODE END PFP */

//* Private user code ---------------------------------------------------------*/
 /* USER CODE BEGIN 0 */



/**
  * @brief USART2 回环测试，从串口助手接收 1 字节并原样回传
  * @note  阻塞 1s 超时，确保主循环仍能执行其他任务
  */
static void debug_uart2_echo_test(void)
{
  uint8_t ch = 0U;
  if (HAL_UART_Receive(&huart2, &ch, 1U, 1000U) == HAL_OK)
  {
    HAL_UART_Transmit(&huart2, &ch, 1U, 1000U);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  // DMA_UART1_Start();
  // DMA_UART2_Start();
   /* USER CODE BEGIN 2 */
  // 1. 初始化底盘缓存与 OLED
  Chassis_Init();
  OLED_Init();
  uint32_t last_telemetry_tick = 0;
  const uint32_t TELEMETRY_PERIOD_MS = 8;
  /* USER CODE END 2 */
  // 2. 初始化 4 个步进电机（假设拨码 ID 分别为 1, 2, 3, 4，接在 USART1 上）
  for (uint8_t i = 0; i < 4; i++) {
      Emm42_Init(&g_emm42_motors[i], &huart1, i + 1);
      Emm42_Enable(&g_emm42_motors[i], true, false); // 使能电机
  }
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Loop();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief 125Hz 核心控制任务：根据模式下发 速度 或 位置 给电机
 */
void Task_ControlAndTelemetry_125Hz(void)
{
    static uint32_t last_seq = 0xFFFFFFFFU;
    static ChassisControlMode last_mode = CHASSIS_MODE_SPEED;
    static bool pending_sync = false;
    static bool stopped = true;

    ChassisSetpoint sp;
    if (!Chassis_GetSetpoint(&sp)) {
        if (!stopped) {
            for (int i = 0; i < 4; i++) {
                Emm42_StopNow(&g_emm42_motors[i], true);
            }
            Emm42_SyncStart(&g_emm42_motors[0]);
            stopped = true;
        }
        return;
    }

    // 超时保护：上位机掉线/丢帧后强制停机
    uint32_t now = HAL_GetTick();
    if ((now - sp.last_update_ms) > CHASSIS_CMD_TIMEOUT_MS) {
        if (!stopped) {
            for (int i = 0; i < 4; i++) {
                Emm42_StopNow(&g_emm42_motors[i], true);
            }
            Emm42_SyncStart(&g_emm42_motors[0]);
            stopped = true;
        }
        return;
    }
    stopped = false;

    bool is_new_cmd = (sp.seq != last_seq) || (sp.mode != last_mode);

    if (is_new_cmd) {
        if (sp.mode == CHASSIS_MODE_SPEED) {
            for (int i = 0; i < 4; i++) {
                float rpm_f = sp.wheel_rad_s[i] * 9.5492966f;
                bool ccw = (rpm_f < 0);
                uint16_t rpm = (uint16_t)fabs(rpm_f);
                if (rpm > 1000) rpm = 1000;
                Emm42_SetSpeed(&g_emm42_motors[i], ccw, rpm, 0, true);
            }
        } 
        else if (sp.mode == CHASSIS_MODE_POSITION) {
            for (int i = 0; i < 4; i++) {
                int32_t p = sp.wheel_pulses[i];
                bool ccw = (p < 0);
                uint32_t pulses = (uint32_t)abs(p);
                uint16_t run_rpm = 200;
                Emm42_MoveRelative(&g_emm42_motors[i], ccw, run_rpm, 0, pulses, false, true);
            }
        }

        pending_sync = true;
        last_seq = sp.seq;
        last_mode = sp.mode;
    }

    if (pending_sync) {
        Emm42_SyncStart(&g_emm42_motors[0]);
        pending_sync = false;
    }
}


/**
 * @brief 200Hz IMU 获取任务
 */
void Task_Imu_200Hz(void)
{
    JY901B_PollI2C(&hi2c2);
}
/**
 * @brief 20Hz OLED 刷新任务 (显示姿态和轮子速度)
 */
void Task_OLED_20Hz(void)
{
    OLED_NewFrame();
    char buf[32];
    
    // 显示 IMU 数据
    JY901B_Data imu;
    if (JY901B_GetSnapshot(&imu)) {
        snprintf(buf, sizeof(buf), "R:%.1f P:%.1f", imu.roll, imu.pitch);
        OLED_PrintASCIIString(0, 0, buf, &afont8x6, OLED_COLOR_NORMAL);
    } else {
        OLED_PrintASCIIString(0, 0, "IMU Disconnected", &afont8x6, OLED_COLOR_NORMAL);
    }
    // 显示底盘下发状态
    ChassisSetpoint sp;
    if (Chassis_GetSetpoint(&sp)) {
        snprintf(buf, sizeof(buf), "W1:%.1f W2:%.1f", sp.wheel_rad_s[0], sp.wheel_rad_s[1]);
        OLED_PrintASCIIString(0, 16, buf, &afont8x6, OLED_COLOR_NORMAL);
    }
    OLED_ShowFrame();
}
/**
 * @brief 1Hz 健康检测任务 (心跳灯)
 */
void Task_Health_1Hz(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
/**
 * @brief 裸机多任务调度器 (在 main 函数的 while(1) 里面调用)
 */
void App_Loop(void)
{
    uint32_t current_tick = HAL_GetTick();
    static uint8_t tx_frame_buffer[128];
    // 200Hz 任务 (每 5ms 执行一次)
    static uint32_t last_imu_tick = 0;
    if (current_tick - last_imu_tick >= 5) {
        last_imu_tick = current_tick;
        Task_Imu_200Hz();
    }
    // 125Hz 任务 (每 8ms 执行一次)
    static uint32_t last_ctrl_tick = 0;
    if (current_tick - last_ctrl_tick >= 8) {
        last_ctrl_tick = current_tick;
        Task_ControlAndTelemetry_125Hz();
        uint8_t frame_len = data_frame_made(tx_frame_buffer);
        send_chassis_data(tx_frame_buffer, frame_len); // DMA 极速异步发送
    }
    // 20Hz 任务 (每 50ms 执行一次)
    static uint32_t last_oled_tick = 0;
    if (current_tick - last_oled_tick >= 50) {
        last_oled_tick = current_tick;
        Task_OLED_20Hz();
    }
    // 1Hz 任务 (每 1000ms 执行一次)
    static uint32_t last_health_tick = 0;
    if (current_tick - last_health_tick >= 1000) {
        last_health_tick = current_tick;
        Task_Health_1Hz();
    }
}

// /**
//  * @brief 定时器更新中断回调。
//  *
//  * @param[in] htim 触发中断的定时器句柄
//  *
//  * 1. 当 TIM1 产生 Update 事件时，将 1kHz 节拍计数加一；
//  * 2. 由 App_Loop 在主循环中消费该计数，实现“定时器中断轻量化，控制逻辑在主循环”的设计；
//  * 3. 其他定时器如 TIM2/3/4 不在此回调中处理，避免干扰 PWM 与舵机输出。
//  */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   if (htim->Instance == TIM1)
//   {
//     g_tick_1khz++;
//   }
// }

// /**
//  * @brief 应用主循环调度函数。
//  * 
//  *
//  * 1. 读取当前 1kHz 节拍计数 g_tick_1khz；
//  * 2. 按“last_tick → current_tick”逐步补齐所有未执行的 1kHz 步长；
//  * 3. 在每个步长中按固定顺序依次执行：
//  *    - Task_Imu_200Hz                 ：按 5 分频以 200Hz 轮询 IMU；
//  *    - Task_ControlAndTelemetry_125Hz：按 8 分频以 125Hz 执行控制 + IMU 上传；
//  *    - Task_OLED_20Hz                ：按 250 分频以 4Hz 轻量刷新 OLED（已优化以释放 CPU 资源给 Telemetry）；
//  *    - Task_Health_1Hz               ：按 1Hz 频率做健康指示。 
//  *
//  * @note 该函数应在 while(1) 主循环中持续调用，不应在中断中调用。
//  */
// void App_Loop(void)
// {
//   //APPloop_Init
//   // oled可视化，可选
//   // uint8_t key_oled = 1;
//   // if (key_oled)
//   // {
//   //   float gx,gy,gz;
//   //   IMUFilter_
//   // }

//   static uint32_t last_tick = 0U;
//   uint32_t current_tick = g_tick_1khz;

//   while (last_tick != current_tick)
//   {
//     last_tick++;
//     Task_Imu_200Hz();
//     Task_ControlAndTelemetry_125Hz();
//     Task_OLED_20Hz();
//     Task_Health_1Hz();
//   }
// }

// /**
//  * @brief 控制回路：消费最新底盘指令并更新 PWM/DIR。
//  *
//  * 1. 从 Chassis 模块读取最近一次有效底盘指令（四轮角速度）；
//  * 2. 若尚未收到任何有效指令，则关闭所有 PWM 输出，避免电机误动；
//  * 3. 对每个轮子：
//  *    - 根据角速度正负决定 DIR 引脚电平；
//  *    - 通过 CHASSIS_PULSE_PER_RAD 将 rad/s 转换为 STEP 频率 (Hz)；
//  *    - 调用 MotorPwm_Apply 写入 PWM 计数器。
//  *
//  * @note 需保持执行时间短、无阻塞操作，由 Task_ControlAndTelemetry_125Hz 触发。
//  */
// static void ControlLoop_Apply(void)
// {
//   ChassisSetpoint setpoint;
//   if (!Chassis_GetSetpoint(&setpoint))
//   {
//     MotorPwm_StopAll();
//     return;
//   }

//   for (uint8_t i = 0U; i < 4U; ++i)
//   {
//     const float target_rad_s = setpoint.wheel_rad_s[i];
//     const bool forward = (target_rad_s >= 0.0f);
//     const float freq_hz = fabsf(target_rad_s) * CHASSIS_PULSE_PER_RAD;
//     MotorPwm_Apply(i, freq_hz, forward);
//   }
// }

// /**
//  * @brief 200Hz IMU 任务：通过 I2C2 轮询 JY901 寄存器窗口。
//  *
//  * 1. 使用简单分频计数器，将 1kHz 节拍分频为 200Hz（IMU_TASK_DIV = 5）；
//  * 2. 在分频到达时调用 JY901B_PollI2C(&hi2c2)，一次读取完整的 32B 寄存器窗口；
//  * 3. JY901B 模块内部完成寄存器解析与 g_jy901_data 更新，并维护错误计数器。
//  *
//  * @note 仅在 App_Loop 中按分频调用，不在中断上下文中直接访问 I2C。
//  */
// void Task_Imu_200Hz(void)
// {
// static uint8_t div = 0U;
//   if (++div < IMU_TASK_DIV)
//   {
//     return;
//   }
//   div = 0U;
//   /* 1. I2C 轮询 IMU，失败计数累加 */
//   (void)JY901B_PollI2C(&hi2c2);
// }

// /**
//  * @brief 125Hz 控制+遥测任务：执行控制回路并在有新样本时推送 IMU 数据。
//  *
//  * 1. 使用分频计数器将 1kHz 节拍分频为 125Hz（CTRL_TELE_TASK_DIV = 8）；
//  * 2. 每次进入任务先执行控制回路（PWM/DIR 更新），随后检查是否有新的 IMU 样本；
//  * 3. 当存在新样本时，将 g_jy901_raw（32B 寄存器窗口）打包为 FUNC_SENSOR/CMD_SENSOR_GET_IMU 帧。
//  *
//  * @note 上位机需立刻运行控制算法并回传命令，串口链路占用约 35%。
//  */
// void Task_ControlAndTelemetry_125Hz(void)
// {
//   static uint8_t div = 0U;
//   static uint32_t last_sample = 0U;

//   if (++div < CTRL_TELE_TASK_DIV)
//   {
//     return;
//   }
//   div = 0U;

//   /* 1. 运行底盘控制回路 */
//   ControlLoop_Apply();

//   /* 2. 检查是否有新的 IMU 样本需要上传 */
//   const uint32_t current_sample = JY901B_GetSampleCounter();
//   if (current_sample == last_sample)
//   {
//     return;
//   }

//   last_sample = current_sample;
//   (void)protocol_send_sensor_payload(CMD_SENSOR_GET_IMU,
//                                      g_jy901_raw,
//                                      (uint16_t)JY901_I2C_BLOCK_BYTES);
// }
// /**
//  * @brief 4Hz OLED 显示任务：低频刷新姿态信息（已优化频率以释放 CPU）。
//  *
//  * 1. 将 1kHz 节拍按 250 分频压缩为 4Hz；
//  * 2. 读取最新 IMU 快照并渲染到 OLED，避免占用控制带宽。
//  */
// void Task_OLED_20Hz(void)
// {
//   static uint8_t div = 0U;
//   static uint32_t last_sample = 0U;
//   if (++div < OLED_TASK_DIV)
//   {
//     return;
//   }
//   div = 0U;
//   /* 1. 周期性回传IMU数据，若无有效样本则跳过 */
//   const uint32_t current_sample = JY901B_GetSampleCounter();
//   if (current_sample == last_sample)
//   {
//     return;
//   }

//   /* 2. 屏幕打印IMU数据 */
//   last_sample = current_sample;

//   /* 获取当前 IMU 数据快照 */
//   JY901B_Data imu_snapshot;
//   if (!JY901B_GetSnapshot(&imu_snapshot))
//   {
//     return; /* 尚未采集到有效数据 */
//   }

//   /* 准备显示缓冲区 */
//   char line_buf[22]; /* 128px / 6px_per_char = 21 chars + '\0' */

//   /* 开始新的一帧 */
//   OLED_NewFrame();

//   /* 第1行: Roll（使用整数度数避免浮点 printf 依赖） */
//   snprintf(line_buf, sizeof(line_buf), "Roll :%6ld", (long)imu_snapshot.roll);
//   OLED_PrintASCIIString(0, 0, line_buf, &afont12x6, OLED_COLOR_NORMAL);

//   /* 第2行: Pitch */
//   snprintf(line_buf, sizeof(line_buf), "Pitch:%6ld", (long)imu_snapshot.pitch);
//   OLED_PrintASCIIString(0, 13, line_buf, &afont12x6, OLED_COLOR_NORMAL);

//   /* 第3行: Yaw */
//   snprintf(line_buf, sizeof(line_buf), "Yaw  :%6ld", (long)imu_snapshot.yaw);
//   OLED_PrintASCIIString(0, 26, line_buf, &afont12x6, OLED_COLOR_NORMAL);

//   /* 第4行: 采样率提示（固定标注 200 Hz 采集 + 125 Hz 上传节奏） */
//   snprintf(line_buf, sizeof(line_buf), "Hz:200/125");
//   OLED_PrintASCIIString(0, 39, line_buf, &afont12x6, OLED_COLOR_NORMAL);

//   /* 第5行: 错误计数 */
//   const uint32_t error_count = JY901B_GetErrorCounter();
//   snprintf(line_buf, sizeof(line_buf), "Err:%lu", error_count);
//   OLED_PrintASCIIString(0, 52, line_buf, &afont12x6, OLED_COLOR_NORMAL);

//   /* 刷新到屏幕 */
//   OLED_ShowFrame();
// }


// /**
//  * @brief 1Hz 健康监控任务：用于简单心跳指示/错误统计。
//  *
//  * 1. 通过分频计数器将 1kHz 节拍压缩为 1Hz；
//  * 2. 每秒翻转一次 PC13 板载 LED，用作“主循环仍在运行”的心跳指示。
//  *
//  * @note 后续可在此任务中加入 IMU 错误计数、UART 错误等健康信息的统计与上报。
//  */
// void Task_Health_1Hz(void)
// {
//   static uint16_t div = 0U;
//   if (++div < HEALTH_TASK_DIV)
//   {
    
//     return;
//   }
//   div = 0U;
//   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
// }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
