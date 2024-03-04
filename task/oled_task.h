//
// Created by 86136 on 2024/2/28.
//

#ifndef MECANUM_CART_OLED_TASK_H
#define MECANUM_CART_OLED_TASK_H

void oled_task();

#define SHOW_TASK_PRIO 3
#define SHOW_STK_SIZE 512

void show_task(void *pvParameters);
void oled_show(void);
void APP_Show(void);
void OLED_ShowCheckConfirming(void);
void OLED_ShowChecking(void);
void OLED_ShowCheckResult(void);

#endif // MECANUM_CART_OLED_TASK_H
