//
// Created by luojihao on 2025/10/19.
//

#include "main.h"
#include "cmsis_os.h"

extern osMessageQueueId_t NumQueueHandle;

#ifndef FREERTOS_LED_NUMADD_TASK_H
#define FREERTOS_LED_NUMADD_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

    void StartNumAddTask(void *argument);

#ifdef __cplusplus
}

#endif

#endif //FREERTOS_LED_NUMADD_TASK_H