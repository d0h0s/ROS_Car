//
// Created by 86136 on 2024/2/28.
//

#ifndef MECANUM_CART_BALANCE_TASK_H
#define MECANUM_CART_BALANCE_TASK_H

#define BALANCE_TASK_PRIO 4  // Task priority //??????????
#define BALANCE_STK_SIZE 512 // Task stack size //?????????¨®??

// Parameter of kinematics analysis of omnidirectional trolley
// ???¨°???????????¡ì¡¤???????
#define X_PARAMETER (sqrt(3) / 2.f)
#define Y_PARAMETER (0.5f)
#define L_PARAMETER (1.0f)

extern short test_num;
extern int robot_mode_check_flag;
extern unsigned char command_lost_count;

void balance_task();
void Balance_task(void *pvParameters);
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert, float low, float high);
int target_limit_int(int insert, int low, int high);
unsigned char Turn_Off(float voltage);
unsigned int myabs(long int a);
int Incremental_PI_A(float Encoder, float Target);
int Incremental_PI_B(float Encoder, float Target);
int Incremental_PI_C(float Encoder, float Target);
int Incremental_PI_D(float Encoder, float Target);
void Get_RC(void);
void Remote_Control(void);
void Drive_Motor(float Vx, float Vy, float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx, float vy, float vz);
void PS2_control(void);
float float_abs(float insert);
void robot_mode_check(void);

void balabce_task();

#endif // MECANUM_CART_BALANCE_TASK_H
