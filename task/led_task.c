//
// Created by 86136 on 2024/2/28.
//

#include "led_task.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

unsigned int Led_Count = 500;

void led_task() {

  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
  while (1) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    
    vTaskDelay(Led_Count);
  }
}
