//
// Created by 86136 on 2024/2/28.
//

#include "balance_task.h"
#include "FreeRTOS.h"
#include "MPU6050_task.h"
#include "encoder.h"
#include "key.h"
#include "main.h"
#include "motor.h"
#include "robot_select_init.h"
#include "system.h"
#include "task.h"
#include "tim.h"
int Time_count = 0; // Time variable //???±±???

void balance_task() {
  motor_pwm_init();   // 电机的pwm初始化
  encoder_tim_init(); // 编码器的pwm初始化
  Robot_Init(MEC_wheelspacing, MEC_axlespacing, 0, HALL_30F, Photoelectric_500,
             Mecanum_75); // 初始化小车的信息
  unsigned int lastWakeTime = HAL_GetTick();
  unsigned char mode = 0;
  while (1) {
    if (Time_count < 3000)
      Time_count++;

    //    Set_Pwm(0, 3000, 0, 0, 0);
    Get_Velocity_Form_Encoder();

    if (APP_ON_Flag)
      Get_RC();
    else if (PS2_ON_Flag)
      PS2_control();
    else {
      Drive_Motor(Move_X, Move_Y, Move_Z);
    }

    Key();

    if (Turn_Off(get_battery_voltage()) == 0) {
      MOTOR_A.Motor_Pwm = Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
      MOTOR_B.Motor_Pwm = Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
      MOTOR_C.Motor_Pwm = Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
      MOTOR_D.Motor_Pwm = Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);

      Limit_Pwm(16700);

      Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm,
              MOTOR_D.Motor_Pwm, 0);
    } else {
      Set_Pwm(0, 0, 0, 0, 0);
    }
    vTaskDelay(10);
  }
}

// Robot mode is wrong to detect flag bits
int robot_mode_check_flag = 0;

short test_num;
unsigned char command_lost_count = 0;

Encoder OriginalEncoder; // Encoder raw data
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed
of each wheel according to the target speed of three axes Input   : X and Y, Z
axis direction of the target movement speed Output  : none
**************************************************************************/
void Drive_Motor(float Vx, float Vy, float Vz) {
  float amplitude = 3.5f;

  Smooth_control(Vx, Vy, Vz);

  Vx = smooth_control.VX;
  Vy = smooth_control.VY;
  Vz = smooth_control.VZ;

  MOTOR_A.Target = +Vy + Vx - Vz * (Axle_spacing + Wheel_spacing);
  MOTOR_B.Target = -Vy + Vx - Vz * (Axle_spacing + Wheel_spacing);
  MOTOR_C.Target = +Vy + Vx + Vz * (Axle_spacing + Wheel_spacing);
  MOTOR_D.Target = -Vy + Vx + Vz * (Axle_spacing + Wheel_spacing);

  // Wheel (motor) target speed limit
  MOTOR_A.Target = target_limit_float(MOTOR_A.Target, -amplitude, amplitude);
  MOTOR_B.Target = target_limit_float(MOTOR_B.Target, -amplitude, amplitude);
  MOTOR_C.Target = target_limit_float(MOTOR_C.Target, -amplitude, amplitude);
  MOTOR_D.Target = target_limit_float(MOTOR_D.Target, -amplitude, amplitude);
}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and
direction Input   : PWM Output  : none
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d, int servo) {
  // Forward and reverse control of motor
  if (motor_a < 0)
    PWMA1 = 16799, PWMA2 = 16799 + motor_a;
  else
    PWMA2 = 16799, PWMA1 = 16799 - motor_a;

  // Forward and reverse control of motor
  if (motor_b < 0)
    PWMB1 = 16799, PWMB2 = 16799 + motor_b;
  else
    PWMB2 = 16799, PWMB1 = 16799 - motor_b;
  //  PWMB1=10000,PWMB2=5000;

  // Forward and reverse control of motor
  if (motor_c < 0)
    PWMC1 = 16799, PWMC2 = 16799 + motor_c;
  else
    PWMC2 = 16799, PWMC1 = 16799 - motor_c;

  // Forward and reverse control of motor
  if (motor_d < 0)
    PWMD1 = 16799, PWMD2 = 16799 + motor_d;
  else
    PWMD2 = 16799, PWMD1 = 16799 - motor_d;

  Servo_PWM = servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : nonerobot_mode_check
**************************************************************************/
void Limit_Pwm(int amplitude) {
  MOTOR_A.Motor_Pwm =
      target_limit_float(MOTOR_A.Motor_Pwm, -amplitude, amplitude);
  MOTOR_B.Motor_Pwm =
      target_limit_float(MOTOR_B.Motor_Pwm, -amplitude, amplitude);
  MOTOR_C.Motor_Pwm =
      target_limit_float(MOTOR_C.Motor_Pwm, -amplitude, amplitude);
  MOTOR_D.Motor_Pwm =
      target_limit_float(MOTOR_D.Motor_Pwm, -amplitude, amplitude);
}
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
????????????·ù????
??????????·ù??
·???  ??????
**************************************************************************/
float target_limit_float(float insert, float low, float high) {
  if (insert < low)
    return low;
  else if (insert > high)
    return high;
  else
    return insert;
}
int target_limit_int(int insert, int low, int high) {
  if (insert < low)
    return low;
  else if (insert > high)
    return high;
  else
    return insert;
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag
status Input   : Voltage Output  : Whether control is allowed, 1: not allowed, 0
allowed ???????????ì?é??????????????????×??????í???§??±ê????×??? ??????????????
·???  ??????·????í??????1???????í??0???í
**************************************************************************/
unsigned char Turn_Off(float voltage) {
  unsigned char temp;
  if (voltage < 10.0f || EN == 0 || Flag_Stop == 1) {
    temp = 1;
    PWMA1 = 0;
    PWMA2 = 0;
    PWMB1 = 0;
    PWMB2 = 0;
    PWMC1 = 0;
    PWMC1 = 0;
    PWMD1 = 0;
    PWMD2 = 0;
  } else
    temp = 0;
  return temp;
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int

**************************************************************************/
unsigned int myabs(long int a) {
  unsigned int temp;
  if (a < 0)
    temp = -a;
  else
    temp = a;
  return temp;
}

int Incremental_PI_A(float Encoder, float Target) {
  static float Bias, Pwm, Last_bias;
  Bias = Target - Encoder; // Calculate the deviation //????????
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
  if (Pwm > 16700)
    Pwm = 16700;
  if (Pwm < -16700)
    Pwm = -16700;
  Last_bias = Bias; // Save the last deviation //±?????????????
  return Pwm;
}
int Incremental_PI_B(float Encoder, float Target) {
  static float Bias, Pwm, Last_bias;
  Bias = Target - Encoder; // Calculate the deviation //????????
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
  if (Pwm > 16700)
    Pwm = 16700;
  if (Pwm < -16700)
    Pwm = -16700;
  Last_bias = Bias; // Save the last deviation //±?????????????
  return Pwm;
}
int Incremental_PI_C(float Encoder, float Target) {
  static float Bias, Pwm, Last_bias;
  Bias = Target - Encoder; // Calculate the deviation //????????
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
  if (Pwm > 16700)
    Pwm = 16700;
  if (Pwm < -16700)
    Pwm = -16700;
  Last_bias = Bias; // Save the last deviation //±?????????????
  return Pwm;
}
int Incremental_PI_D(float Encoder, float Target) {
  static float Bias, Pwm, Last_bias;
  Bias = Target - Encoder; // Calculate the deviation //????????
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
  if (Pwm > 16700)
    Pwm = 16700;
  if (Pwm < -16700)
    Pwm = -16700;
  Last_bias = Bias; // Save the last deviation //±?????????????
  return Pwm;
}

void Get_RC(void) {
  unsigned char Flag_Move = 1;
  if (Car_Mode == Mec_Car ||
      Car_Mode == Omni_Car) // The omnidirectional wheel moving trolley can move
  {
    switch (Flag_Direction) // Handle direction control commands
    {
    case 1:
      Move_X = RC_Velocity;
      Move_Y = 0;
      Flag_Move = 1;
      break;
    case 2:
      Move_X = RC_Velocity;
      Move_Y = -RC_Velocity;
      Flag_Move = 1;
      break;
    case 3:
      Move_X = 0;
      Move_Y = -RC_Velocity;
      Flag_Move = 1;
      break;
    case 4:
      Move_X = -RC_Velocity;
      Move_Y = -RC_Velocity;
      Flag_Move = 1;
      break;
    case 5:
      Move_X = -RC_Velocity;
      Move_Y = 0;
      Flag_Move = 1;
      break;
    case 6:
      Move_X = -RC_Velocity;
      Move_Y = RC_Velocity;
      Flag_Move = 1;
      break;
    case 7:
      Move_X = 0;
      Move_Y = RC_Velocity;
      Flag_Move = 1;
      break;
    case 8:
      Move_X = RC_Velocity;
      Move_Y = RC_Velocity;
      Flag_Move = 1;
      break;
    default:
      Move_X = 0;
      Move_Y = 0;
      Flag_Move = 0;
      break;
    }
    if (Flag_Move == 0) {

      if (Flag_Left == 1)
        Move_Z = PI / 2 * (RC_Velocity / 500);
      else if (Flag_Right == 1)
        Move_Z = -PI / 2 * (RC_Velocity / 500);
      else
        Move_Z = 0;
    }
  } else {
    switch (Flag_Direction) {
    case 1:
      Move_X = +RC_Velocity;
      Move_Z = 0;
      break;
    case 2:
      Move_X = +RC_Velocity;
      Move_Z = -PI / 2;
      break;
    case 3:
      Move_X = 0;
      Move_Z = -PI / 2;
      break;
    case 4:
      Move_X = -RC_Velocity;
      Move_Z = -PI / 2;
      break;
    case 5:
      Move_X = -RC_Velocity;
      Move_Z = 0;
      break;
    case 6:
      Move_X = -RC_Velocity;
      Move_Z = +PI / 2;
      break;
    case 7:
      Move_X = 0;
      Move_Z = +PI / 2;
      break;
    case 8:
      Move_X = +RC_Velocity;
      Move_Z = +PI / 2;
      break;
    default:
      Move_X = 0;
      Move_Z = 0;
      break;
    }
    if (Flag_Left == 1)
      Move_Z = PI / 2;
    else if (Flag_Right == 1)
      Move_Z = -PI / 2;
  }

  if (Car_Mode == Akm_Car) {

    Move_Z = Move_Z * 2 / 9;
  } else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car ||
             Car_Mode == FourWheel_Car) {
    if (Move_X < 0)
      Move_Z = -Move_Z;
    Move_Z = Move_Z * RC_Velocity / 500;
  }

  Move_X = Move_X / 1000;
  Move_Y = Move_Y / 1000;
  Move_Z = Move_Z;

  Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
????????????PS2??±ú?????ü?????????í
????????????
·???  ??????
**************************************************************************/
void PS2_control(void) {
  int LX, LY, RY;
  int Threshold = 20; // Threshold to ignore small movements of the joystick
                      // //????????????????·ù????×÷

  // 128 is the median.The definition of X and Y in the PS2 coordinate system is
  // different from that in the ROS coordinate system
  // 128????????PS2×?±ê????ROS×?±ê????X??Y???¨???????ù
  LY = -(PS2_LX - 128);
  LX = -(PS2_LY - 128);
  RY = -(PS2_RX - 128);

  // Ignore small movements of the joystick //??????????·ù????×÷
  if (LX > -Threshold && LX < Threshold)
    LX = 0;
  if (LY > -Threshold && LY < Threshold)
    LY = 0;
  if (RY > -Threshold && RY < Threshold)
    RY = 0;

  if (PS2_KEY == 11)
    RC_Velocity += 5; // To accelerate//????
  else if (PS2_KEY == 9)
    RC_Velocity -= 5; // To slow down //????

  if (RC_Velocity < 0)
    RC_Velocity = 0;

  // Handle PS2 controller control commands
  // ??PS2??±ú?????ü?????????í
  Move_X = LX * RC_Velocity / 128;
  Move_Y = LY * RC_Velocity / 128;
  Move_Z = RY * (PI / 2) / 128;

  // Z-axis data conversion //Z?á????×???
  if (Car_Mode == Mec_Car || Car_Mode == Omni_Car) {
    Move_Z = Move_Z * RC_Velocity / 500;
  } else if (Car_Mode == Akm_Car) {
    // Ackermann structure car is converted to the front wheel steering Angle
    // system target value, and kinematics analysis is pearformed
    // °????ü?á??????×??????°??×??ò????
    Move_Z = Move_Z * 2 / 9;
  } else if (Car_Mode == Diff_Car || Car_Mode == Tank_Car ||
             Car_Mode == FourWheel_Car) {
    if (Move_X < 0)
      Move_Z = -Move_Z; // The differential control principle series requires
                        // this treatment //???????????í?????è???????í
    Move_Z = Move_Z * RC_Velocity / 500;
  }

  // Unit conversion, mm/s -> m/s
  // ????×?????mm/s -> m/s
  Move_X = Move_X / 1000;
  Move_Y = Move_Y / 1000;
  Move_Z = Move_Z;

  // Control target value is obtained and kinematics analysis is performed
  // ??????????±ê?????????????§·???
  Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none

**************************************************************************/
void Key(void) {
  unsigned char tmp;
  tmp = click_N_Double_MPU6050(50);
  if (tmp == 2)
    memcpy(Deviation_gyro, Original_gyro, sizeof(gyro)),
        memcpy(Deviation_accel, Original_accel, sizeof(accel));
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
??????????????±à???÷????????????????????????m/s
????????????
·???  ??????
**************************************************************************/
void Get_Velocity_Form_Encoder(void) {

  float Encoder_A_pr, Encoder_B_pr, Encoder_C_pr, Encoder_D_pr;
  OriginalEncoder.A = Read_Encoder(2);
  OriginalEncoder.B = Read_Encoder(3);
  OriginalEncoder.C = Read_Encoder(4);
  OriginalEncoder.D = Read_Encoder(5);

  Encoder_A_pr = OriginalEncoder.A;
  Encoder_B_pr = OriginalEncoder.B;
  Encoder_C_pr = -OriginalEncoder.C;
  Encoder_D_pr = -OriginalEncoder.D;

  MOTOR_A.Encoder =
      Encoder_A_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
  MOTOR_B.Encoder =
      Encoder_B_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
  MOTOR_C.Encoder =
      Encoder_C_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
  MOTOR_D.Encoder =
      Encoder_D_pr * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
???????????????á??±ê????×????????í
?????????????á??±ê????
·???  ??????
**************************************************************************/
void Smooth_control(float vx, float vy, float vz) {
  float step = 0.01;

  if (vx > 0)
    smooth_control.VX += step;
  else if (vx < 0)
    smooth_control.VX -= step;
  else if (vx == 0)
    smooth_control.VX = smooth_control.VX * 0.9f;

  if (vy > 0)
    smooth_control.VY += step;
  else if (vy < 0)
    smooth_control.VY -= step;
  else if (vy == 0)
    smooth_control.VY = smooth_control.VY * 0.9f;

  if (vz > 0)
    smooth_control.VZ += step;
  else if (vz < 0)
    smooth_control.VZ -= step;
  else if (vz == 0)
    smooth_control.VZ = smooth_control.VZ * 0.9f;

  smooth_control.VX =
      target_limit_float(smooth_control.VX, -float_abs(vx), float_abs(vx));
  smooth_control.VY =
      target_limit_float(smooth_control.VY, -float_abs(vy), float_abs(vy));
  smooth_control.VZ =
      target_limit_float(smooth_control.VZ, -float_abs(vz), float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
??????????????????????????????
????????????????
·???  ??????????????????
**************************************************************************/
float float_abs(float insert) {
  if (insert >= 0)
    return insert;
  else
    return -insert;
}
/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in
initialization error caused by the motor spinning.Out of service Input   : none
Output  : none
??????????·????????÷???í???????????????????í??·????ú??×?????????????
????????????
·???  ??????
**************************************************************************/
void robot_mode_check(void) {
  static unsigned char error = 0;

  if (abs(MOTOR_A.Motor_Pwm) > 2500 || abs(MOTOR_B.Motor_Pwm) > 2500 ||
      abs(MOTOR_C.Motor_Pwm) > 2500 || abs(MOTOR_D.Motor_Pwm) > 2500)
    error++;
  // If the output is close to full amplitude for 6 times in a row, it is judged
  // that the motor rotates wildly and makes the motor incapacitated
  // ????????6?????ü?ú·ù???????????????ú??×????????ú?§??
  //  if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;
  if (error > 6)
    Flag_Stop = 1, robot_mode_check_flag = 1;
}