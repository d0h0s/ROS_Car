//
// Created by 86136 on 2024/2/28.
//

#ifndef MECANUM_CART_MOTOR_H
#define MECANUM_CART_MOTOR_H

#include "tim.h"

#define PWM_PORTA1 GPIOB      // PWMA
#define PWM_PIN_A1 GPIO_Pin_8 // PWMA
#define PWMA1 TIM10->CCR1     // PWMA

#define PWM_PORTA2 GPIOB      // PWMA
#define PWM_PIN_A2 GPIO_Pin_9 // PWMA
#define PWMA2 TIM11->CCR1     // PWMA
/*------------------------------------*/

/*--------Motor_B control pins--------*/
#define PWM_PORTB1 GPIOE      // PWMB
#define PWM_PIN_B1 GPIO_Pin_5 // PWMB
#define PWMB1 TIM9->CCR1      // PWMB

#define PWM_PORTB2 GPIOE      // PWMB
#define PWM_PIN_B2 GPIO_Pin_6 // PWMB
#define PWMB2 TIM9->CCR2      // PWMB

/*------------------------------------*/

/*--------Motor_C control pins--------*/
#define PWM_PORTC1 GPIOE       // PWMC
#define PWM_PIN_C1 GPIO_Pin_11 // PWMC
#define PWMC1 TIM1->CCR2       // PWMC

#define PWM_PORTC2 GPIOE      // PWMC
#define PWM_PIN_C2 GPIO_Pin_9 // PWMC
#define PWMC2 TIM1->CCR1      // PWMC

/*------------------------------------*/

/*--------Motor_D control pins--------*/
#define PWM_PORTD1 GPIOE       // PWMD
#define PWM_PIN_D1 GPIO_Pin_14 // PWMD
#define PWMD1 TIM1->CCR4       // PWMD

#define PWM_PORTD2 GPIOE       // PWMD
#define PWM_PIN_D2 GPIO_Pin_13 // PWMD
#define PWMD2 TIM1->CCR3       // PWMD

/*------------------------------------*/
#define EN HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3)

#define Servo_PWM TIM12->CCR2
#define SERVO_INIT 1500 // Servo zero point //???¨²????

void Enable_Pin(void);
void motor_pwm_init();
#endif // MECANUM_CART_MOTOR_H
