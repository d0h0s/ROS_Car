//
// Created by 86136 on 2024/2/28.
//

#ifndef MECANUM_CART_LED_TASK_H
#define MECANUM_CART_LED_TASK_H

#define Buzzer(GPIO_STATE) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_STATE)

extern unsigned int Led_Count;

void led_task();

#endif // MECANUM_CART_LED_TASK_H
