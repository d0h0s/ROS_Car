//
// Created by 86136 on 2024/2/28.
//

#include "data_task.h"
#include "FreeRTOS.h"
#include "MPU6050_task.h"
#include "main.h"
#include "system.h"
#include "task.h"
#include "usart.h"
#include "usartx.h"
unsigned int bluetooth = 0;
unsigned char bluetooth_flag = 0;
void data_task() {
  uart1_init();
  uart2_init();
  uart3_init();
  uart5_init();

  unsigned int lastWakeTime = HAL_GetTick();

  while (1) {
    //    vTaskDelayUntil((TickType_t *)&lastWakeTime, F2T(RATE_20_HZ));

    data_transition();
    //    USART1_SEND(); // Serial port 1 sends data
    USART3_SEND(); // Serial port 3 (ROS) sends data
    //                   //    USART5_SEND(); // Serial port 5 sends data
    printf("%f,%f,%f,%f\n", Move_X, Move_Y, Move_Z, MOTOR_D.Motor_Pwm);
    //    printf("%f,%f,%f,%f\n", MOTOR_A.Encoder, MOTOR_B.Encoder,
    //    MOTOR_C.Encoder,
    //           MOTOR_D.Encoder);
    //    printf("%f,%f\n", get_CCD1_voltage(), get_CCD2_voltage());
    vTaskDelay(50);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    usart1_receive_data();
  } else if (huart == &huart2) {
    usart2_receive_data();
  } else if (huart == &huart3) {
    usart3_receive_data();
  } else if (huart == &huart5) {
    usart5_receive_data();
  }
}

int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return 0;
}