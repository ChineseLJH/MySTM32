//
// Created by luojihao on 2025/10/17.
//

#include "../Inc/led_task.h"

void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {

    uint8_t Num;
    osMessageQueueGet(NumQueueHandle, &Num, 0, osWaitForever);
    for (uint8_t i = 0; i < Num; i++) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      osDelay(100);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      osDelay(100);
    }
    osDelay(2000-200*Num);
  }
  /* USER CODE END StartLedTask */
}