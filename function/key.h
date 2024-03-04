#ifndef __KEY_H
#define __KEY_H
#include "gpio.h"
#include "main.h"

unsigned char click(void);
void Delay_ms(void);
unsigned char click_N_Double(unsigned char time);
unsigned char click_N_Double_MPU6050(unsigned char time);
unsigned char Long_Press(void);

/*--------KEY control pin--------*/
#define KEY_PORT GPIOE
#define KEY_PIN GPIO_Pin_0
#define KEY HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)
/*----------------------------------*/

#endif
