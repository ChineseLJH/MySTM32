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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

#define APPLICATION_START_ADDRESS 0x08004000U // 应用程序的起始地址
#define APPLICATION_END_ADDRESS 0x0800FFFFU   // 应用程序的结束地址

// 1. 定义函数指针类型：无参数，无返回值
typedef void (*pFunction)(void);

// 2. 定义跳转变量
pFunction JumpToApplication;
uint32_t JumpAddress;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Go_To_App(void)
{
  // 1. 检查 App 地址是否合法 (检查栈顶指针是否在 RAM 范围内)
  // STM32F1 的 RAM 通常从 0x20000000 开始
  if (((*(uint32_t *)APPLICATION_START_ADDRESS) & 0x2FFE0000) == 0x20000000)
  {
    // [A] 关中断，防止跳转途中被打断
    __disable_irq();

    // [B] 准备跳转地址 (从 App 的中断向量表里取复位中断地址)
    JumpAddress = *(uint32_t *)(APPLICATION_START_ADDRESS + 4);
    JumpToApplication = (pFunction)JumpAddress;

    // [C] 设置主堆栈指针 (MSP)
    __set_MSP(*(uint32_t *)APPLICATION_START_ADDRESS);

    // [D] 彻底复位外设 (这步很重要，把车清理干净给主程序)
    HAL_RCC_DeInit();
    HAL_DeInit();

    // [E] 重新开中断 (交给 App 去处理)
    __enable_irq();

    // [F] 跳转！
    JumpToApplication();
  }
}

uint8_t Get_Low_Pin_Count(void)
{
  uint8_t num = 0;

  // 统计有多少个引脚是低电平
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    num++;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
    num++;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)
    num++;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET)
    num++;

  return num;
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
  // MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // === 1. 逻辑判断：PA0~PA3 任意两个被拉低 ===
  uint8_t low_count = Get_Low_Pin_Count();

  // 如果低电平数量 >= 2，进入 DFU 模式
  if (low_count >= 2)
  {
    // [DFU 模式]

    // A. 这时候才初始化 USB！
    MX_USB_DEVICE_Init();

    // B. 进入死循环，闪烁 PC13
    while (1)
    {
      low_count = Get_Low_Pin_Count();

      if(low_count == 3)
      {
        Go_To_App();
      }

      // 假设 PC13 是低电平点亮，这里取反即可
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

      // 延时 200ms (USB 是靠中断工作的，这里延时不会影响烧录)
      HAL_Delay(200);
    }
  }
  else
  {
    // [正常模式]

    // A. 不要初始化 USB！

    // B. 直接跳转
    Go_To_App();

    // 如果代码跑到这里，说明跳转失败（没烧程序），
    // 为了防止跑飞，可以也进入一个死循环或闪烁报错
    MX_USB_DEVICE_Init();
    
    while (1)
    {
      // 可以做个快闪，表示 "没有主程序"
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(50);
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
