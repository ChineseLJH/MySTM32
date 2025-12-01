//
// Created by luojihao on 2025/10/19.
//

#include "../Inc/numadd_task.h"

void StartNumAddTask(void *argument)
{
  /* USER CODE BEGIN StartNumAddTask */
  /* Infinite loop */
  for(;;)
  {
    for (int i = 1; i <= 5; i++) {
      osMessageQueuePut(NumQueueHandle, &i, 0, osWaitForever);
      osDelay(2000);
    }
  }
  /* USER CODE END StartNumAddTask */
}