/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"
#include "usart.h"
#include "dma.h"
#include "i2c.h"
#include "can.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_imu.h"
#include "app_chassis.h"
#include "app_odom.h"
#include "app_pose.h"
#include "servo_bus.h"
#include "bsp_debug.h"
#include "app_action.h"
#include "user_plan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * 实际使用的时钟源：0 = HSE（板上晶振），1 = HSI（HSE 起不来时的回退）
 * 调试时可直接观察这个变量确认是否真的用上了晶振。
 */
volatile uint8_t g_clock_source = 0u;
volatile uint8_t g_hse_mhz      = 0u;   /* 实际用上的 HSE 频率 (MHz)，0 = 没用上晶振 */

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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  {
    /* 上电先报时钟：clock=HSE(8MHz) 表示识别到 8MHz 晶振；HSI-fallback 表示晶振没识别出来 */
    Debug_Printf("[SYS] clock=%s(%uMHz) sysclk=%luHz pclk1=%luHz\r\n",
                 (g_clock_source == 0u) ? "HSE" : "HSI-fallback",
                 (unsigned)g_hse_mhz,
                 (unsigned long)HAL_RCC_GetHCLKFreq(),
                 (unsigned long)HAL_RCC_GetPCLK1Freq());
  }
  AppIMU_Init();
  AppOdom_Init();
  Pose_Init();             /* ★世界坐标：以当前里程计和朝向为零点 */
  ServoBus_Init();
  Chassis_Init();
  Act_Init();              /* ★动作层：原子动作接口 */
  Plan_Init();             /* ★你的规划初始化 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- 原有行为已停用 ----
     *   ServoArm_Poll();  示教 / 回放（servo_arm.c 已移出编译，源码在 App/_attic/）
     *   Chassis_Task() 的自动行为（航向保持 / 自动回原点）已在速度模式下不再生效
     */

    /* 传感器采样（保留：Pose 用的是它们的数据） */
    AppIMU_Update();
    AppOdom_Update();
    Pose_Update();           /* ★融合陀螺仪 → 世界坐标 */

    Chassis_Task();          /* ★底盘驱动：把速度指令变成 RS485 帧发出去（必须每拍调） */

    ServoBus_Poll();         /* ★收舵机位置回传，更新 Act_ArmAngle0..3 / Act_GripAngle */

    /* 动作层 + 你的规划 */
    Act_Task();
    Plan_Task();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /*
   * 目标：SYSCLK 168 MHz（HSE 8MHz + PLL：M=4 -> 2MHz，N=168 -> 336MHz，P=2 -> 168MHz）
   *       AHB = 168 MHz，APB1 = 42 MHz，APB2 = 84 MHz，Flash 5 等待周期
   * 为什么必须是 168MHz / APB1=42MHz：CAN 1Mbps 的位时序
   * （Prescaler 6, BS1 4TQ, BS2 2TQ）是按 APB1=42MHz 算的，
   * 时钟不对 CAN 波特率就不对，总线舵机不会应答。
   *
   * 优先用 HSE（外部晶振）；若晶振起不来（没焊/频率不符/HAL 超时），
   * 自动回退到 HSI 16MHz + 同样 PLL 参数，仍得到 168 MHz，
   * 避免板子因晶振问题直接卡死在 Error_Handler。
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;      /* 8MHz / 4 = 2MHz VCO 输入（与原项目一致） */
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  /*
   * 按工程内《Schematic.pdf》：主芯片时钟晶振 X1 = 8MHz（配 22pF 电容）。
   * PLL 参数与原项目（可用版本）完全一致：
   *   PLLM = 4  -> VCO 输入 = 8MHz/4 = 2MHz（规格范围 1~2MHz 的中点，比 1MHz 更稳）
   *   PLLN = 168 -> VCO 输出 = 336MHz
   *   PLLP = 2   -> SYSCLK = 168MHz，APB1 = 42MHz（CAN 1Mbps 需要）
   */
  g_hse_mhz = 0u;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK)
  {
    g_hse_mhz = 8u;                 /* 按原理图：8MHz 晶振 */
  }

  g_clock_source = 0u;
  if (g_hse_mhz == 0u)
  {
    /* HSE 不可用：回退 HSI 16MHz，PLLM=8 -> VCO 输入 2MHz -> ×168 -> /2 = 168MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    g_clock_source = 1u;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /* 168MHz 下必须打开 Flash 预取与缓存，否则取指会被 Flash 拖慢 */
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}

/* USER CODE BEGIN 4 */

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
