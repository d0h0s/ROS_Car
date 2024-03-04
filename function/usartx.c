#include "usartx.h"
#include "robot_select_init.h"
#include "usart.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void) {
  Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; // Frame_header //帧头
  Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     // Frame_tail //帧尾

  Send_Data.Sensor_Str.X_speed =
      ((MOTOR_A.Encoder + MOTOR_B.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) /
       4) *
      1000;
  Send_Data.Sensor_Str.Y_speed =
      ((MOTOR_A.Encoder - MOTOR_B.Encoder + MOTOR_C.Encoder - MOTOR_D.Encoder) /
       4) *
      1000;
  Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder - MOTOR_B.Encoder +
                                   MOTOR_C.Encoder + MOTOR_D.Encoder) /
                                  4 / (Axle_spacing + Wheel_spacing)) *
                                 1000;

  // The acceleration of the triaxial acceleration //加速度计三轴加速度
  Send_Data.Sensor_Str.Accelerometer.X_data =
      accel[1]; // The accelerometer Y-axis is converted to the ros coordinate X
                // axis //加速度计Y轴转换到ROS坐标X轴
  Send_Data.Sensor_Str.Accelerometer.Y_data =
      -accel[0]; // The accelerometer X-axis is converted to the ros coordinate
                 // y axis //加速度计X轴转换到ROS坐标Y轴
  Send_Data.Sensor_Str.Accelerometer.Z_data =
      accel[2]; // The accelerometer Z-axis is converted to the ros coordinate Z
                // axis //加速度计Z轴转换到ROS坐标Z轴

  // The Angle velocity of the triaxial velocity //角速度计三轴角速度
  Send_Data.Sensor_Str.Gyroscope.X_data =
      gyro[1]; // The Y-axis is converted to the ros coordinate X axis
               // //角速度计Y轴转换到ROS坐标X轴
  Send_Data.Sensor_Str.Gyroscope.Y_data =
      -gyro[0]; // The X-axis is converted to the ros coordinate y axis
                // //角速度计X轴转换到ROS坐标Y轴
  if (Flag_Stop == 0)
    // If the motor control bit makes energy state, the z-axis velocity is sent
    // normall
    // 如果电机控制位使能状态，那么正常发送Z轴角速度
    Send_Data.Sensor_Str.Gyroscope.Z_data = gyro[2];
  else
    // If the robot is static (motor control dislocation), the z-axis is 0
    // 如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0
    Send_Data.Sensor_Str.Gyroscope.Z_data = 0;

  // Battery voltage (this is a thousand times larger floating point number,
  // which will be reduced by a thousand times as well as receiving the data).
  // 电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
  Send_Data.Sensor_Str.Power_Voltage = (short)(get_battery_voltage() * 1000);
  Send_Data.Sensor_Str.CCD1 = (short)(get_CCD1_voltage() * 1000);
  Send_Data.Sensor_Str.CCD2 = (short)(get_CCD2_voltage() * 1000);

  Send_Data.buffer[0] = Send_Data.Sensor_Str.Frame_Header; // Frame_heade //帧头
  Send_Data.buffer[1] =
      Flag_Stop; // Car software loss marker //小车软件失能标志位

  // The three-axis speed of / / car is split into two eight digit Numbers
  // 小车三轴速度,各轴都拆分为两个8位数据再发送
  Send_Data.buffer[2] = Send_Data.Sensor_Str.X_speed >> 8;
  Send_Data.buffer[3] = Send_Data.Sensor_Str.X_speed;
  Send_Data.buffer[4] = Send_Data.Sensor_Str.Y_speed >> 8;
  Send_Data.buffer[5] = Send_Data.Sensor_Str.Y_speed;
  Send_Data.buffer[6] = Send_Data.Sensor_Str.Z_speed >> 8;
  Send_Data.buffer[7] = Send_Data.Sensor_Str.Z_speed;

  // The acceleration of the triaxial axis of / / imu accelerometer is divided
  // into two eight digit reams
  // IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
  Send_Data.buffer[8] = Send_Data.Sensor_Str.Accelerometer.X_data >> 8;
  Send_Data.buffer[9] = Send_Data.Sensor_Str.Accelerometer.X_data;
  Send_Data.buffer[10] = Send_Data.Sensor_Str.Accelerometer.Y_data >> 8;
  Send_Data.buffer[11] = Send_Data.Sensor_Str.Accelerometer.Y_data;
  Send_Data.buffer[12] = Send_Data.Sensor_Str.Accelerometer.Z_data >> 8;
  Send_Data.buffer[13] = Send_Data.Sensor_Str.Accelerometer.Z_data;

  // The axis of the triaxial velocity of the / /imu is divided into two eight
  // digits IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
  Send_Data.buffer[14] = Send_Data.Sensor_Str.Gyroscope.X_data >> 8;
  Send_Data.buffer[15] = Send_Data.Sensor_Str.Gyroscope.X_data;
  Send_Data.buffer[16] = Send_Data.Sensor_Str.Gyroscope.Y_data >> 8;
  Send_Data.buffer[17] = Send_Data.Sensor_Str.Gyroscope.Y_data;
  Send_Data.buffer[18] = Send_Data.Sensor_Str.Gyroscope.Z_data >> 8;
  Send_Data.buffer[19] = Send_Data.Sensor_Str.Gyroscope.Z_data;

  // Battery voltage, split into two 8 digit Numbers
  // 电池电压,拆分为两个8位数据发送
  Send_Data.buffer[20] = Send_Data.Sensor_Str.Power_Voltage >> 8;
  Send_Data.buffer[21] = Send_Data.Sensor_Str.Power_Voltage;
  Send_Data.buffer[22] = Send_Data.Sensor_Str.CCD1 >> 8;
  Send_Data.buffer[23] = Send_Data.Sensor_Str.CCD1;
  Send_Data.buffer[24] = Send_Data.Sensor_Str.CCD2 >> 8;
  Send_Data.buffer[25] = Send_Data.Sensor_Str.CCD2;

  // Data check digit calculation, Pattern 1 is a data check
  // 数据校验位计算，模式1是发送数据校验
  Send_Data.buffer[26] = Check_Sum(26, 1);

  Send_Data.buffer[27] = Send_Data.Sensor_Str.Frame_Tail; // Frame_tail //帧尾
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void) {
  HAL_UART_Transmit(&huart1, Send_Data.buffer, SEND_DATA_SIZE, HAL_MAX_DELAY);
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART3_SEND(void) {
  HAL_UART_Transmit(&huart3, Send_Data.buffer, SEND_DATA_SIZE, HAL_MAX_DELAY);
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
函数功能：串口5发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART5_SEND(void) {
  HAL_UART_Transmit(&huart5, Send_Data.buffer, SEND_DATA_SIZE, HAL_MAX_DELAY);
}

unsigned char Usart1_Receive;
int Usart2_Receive;
unsigned char Usart3_Receive;
unsigned char Usart5_Receive;

void uart1_init() { HAL_UART_Receive_IT(&huart1, &Usart1_Receive, 1); }

void uart2_init() {
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&Usart2_Receive, 4);
}
void uart3_init() { HAL_UART_Receive_IT(&huart3, &Usart3_Receive, 1); }
void uart5_init() { HAL_UART_Receive_IT(&huart5, &Usart5_Receive, 1); }

int usart1_receive_data(void) {

  static unsigned char Count;
  static unsigned char rxbuf[11];
  int check = 0, error = 1, i;

  HAL_UART_Receive_IT(&huart1, &Usart1_Receive, 1);
  if (Time_count < CONTROL_DELAY)
    // Data is not processed until 10 seconds after startup
    // 开机10秒前不处理数据
    return 0;

  // Fill the array with serial data
  // 串口数据填入数组
  rxbuf[Count] = Usart1_Receive;

  // Ensure that the first data in the array is FRAME_HEADER
  // 确保数组第一个数据为FRAME_HEADER
  if (Usart1_Receive == FRAME_HEADER || Count > 0)
    Count++;
  else
    Count = 0;

  if (Count == 11) // Verify the length of the packet //验证数据包的长度
  {
    Count = 0; // Prepare for the serial port data to be refill into the array
               // //为串口数据重新填入数组做准备
    if (rxbuf[10] ==
        FRAME_TAIL) // Verify the frame tail of the packet //验证数据包的帧尾
    {
      for (i = 0; i < 9; i++) {
        // XOR bit check, used to detect data error
        // 异或位校验，用于检测数据是否出错
        check = rxbuf[i] ^ check;
      }
      if (check == rxbuf[9])
        // XOR bit check successful
        // 异或位校验成功
        error = 0;

      if (error == 0) {
        float Vz;
        if (Usart1_ON_Flag == 0) {
          // Serial port 1 controls flag position 1, other flag position 0
          // 串口1控制标志位置1，其它标志位置0
          // Usart_ON_Flag=1;
          Usart1_ON_Flag = 1;
          APP_ON_Flag = 0;
          PS2_ON_Flag = 0;
          Remote_ON_Flag = 0;
          CAN_ON_Flag = 0;
        }
        command_lost_count = 0;
        // Calculate the 3-axis target velocity from the serial data, which is
        // divided into 8-bit high and 8-bit low units mm/s
        // 从串口数据求三轴目标速度，分高8位和低8位 单位mm/s
        Move_X = XYZ_Target_Speed_transition(rxbuf[3], rxbuf[4]);
        Move_Y = XYZ_Target_Speed_transition(rxbuf[5], rxbuf[6]);
        Vz = XYZ_Target_Speed_transition(rxbuf[7], rxbuf[8]);
        if (Car_Mode == Akm_Car) {
          Move_Z = Vz_to_Akm_Angle(Move_X, Vz);
        } else {
          Move_Z = XYZ_Target_Speed_transition(rxbuf[7], rxbuf[8]);
        }
      }
    }
  }

  return 0;
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
int usart2_receive_data(void) {

  static unsigned char Flag_PID, i, j, Receive[50], Last_Usart2_Receive;
  static float Data;

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&Usart2_Receive, 4);

  if (Deviation_Count < CONTROL_DELAY)
    // Data is not processed until 10 seconds after startup
    // 开机10秒前不处理数据
    return 0;

  if (Usart2_Receive == 0x41 && Last_Usart2_Receive == 0x41 && APP_ON_Flag == 0)
    // 10 seconds after startup, press the forward button of APP to enter APP
    // control mode
    // The APP controls the flag position 1 and the other flag position 0
    // 开机10秒之后，按下APP的前进键进入APP控制模式
    // APP控制标志位置1，其它标志位置0
    PS2_ON_Flag = 0, Remote_ON_Flag = 0, APP_ON_Flag = 1, CAN_ON_Flag = 0,
    Usart1_ON_Flag = 0, Usart5_ON_Flag = 0;
  Last_Usart2_Receive = Usart2_Receive;

  if (Usart2_Receive == 0x4B)
    // Enter the APP steering control interface
    // 进入APP转向控制界面
    Turn_Flag = 1;
  else if (Usart2_Receive == 0x49 || Usart2_Receive == 0x4A)
    // Enter the APP direction control interface
    // 进入APP方向控制界面
    Turn_Flag = 0;

  if (Turn_Flag == 0) {
    // App rocker control interface command
    // APP摇杆控制界面命令
    if (Usart2_Receive >= 0x41 && Usart2_Receive <= 0x48) {
      Flag_Direction = Usart2_Receive - 0x40;
    } else if (Usart2_Receive <= 8) {
      Flag_Direction = Usart2_Receive;
    } else
      Flag_Direction = 0;
  } else if (Turn_Flag == 1) {
    // APP steering control interface command
    // APP转向控制界面命令
    if (Usart2_Receive == 0x43)
      Flag_Left = 0, Flag_Right = 1; // Right rotation //右自转
    else if (Usart2_Receive == 0x47)
      Flag_Left = 1, Flag_Right = 0; // Left rotation  //左自转
    else
      Flag_Left = 0, Flag_Right = 0;
    if (Usart2_Receive == 0x41 || Usart2_Receive == 0x45)
      Flag_Direction = Usart2_Receive - 0x40;
    else
      Flag_Direction = 0;
  }
  if (Usart2_Receive == 0x58)
    RC_Velocity =
        RC_Velocity + 100; // Accelerate the keys, +100mm/s //加速按键，+100mm/s
  if (Usart2_Receive == 0x59)
    RC_Velocity =
        RC_Velocity - 100; // Slow down buttons,   -100mm/s //减速按键，-100mm/s

  // The following is the communication with the APP debugging interface
  // 以下是与APP调试界面通讯
  if (Usart2_Receive == 0x7B)
    Flag_PID = 1; // The start bit of the APP parameter instruction
                  // //APP参数指令起始位
  if (Usart2_Receive == 0x7D)
    Flag_PID =
        2; // The APP parameter instruction stops the bit //APP参数指令停止位

  if (Flag_PID == 1) // Collect data //采集数据
  {
    Receive[i] = Usart2_Receive;
    i++;
  }
  if (Flag_PID == 2) // Analyze the data //分析数据
  {
    if (Receive[3] == 0x50)
      PID_Send = 1;
    else if (Receive[1] != 0x23) {
      for (j = i; j >= 4; j--) {
        Data += (Receive[j - 1] - 48) * pow(10, i - j);
      }
      switch (Receive[1]) {
      case 0x30:
        RC_Velocity = Data;
        break;
      case 0x31:
        Velocity_KP = Data;
        break;
      case 0x32:
        Velocity_KI = Data;
        break;
      case 0x33:
        break;
      case 0x34:
        break;
      case 0x35:
        break;
      case 0x36:
        break;
      case 0x37:
        break;
      case 0x38:
        break;
      }
    }
    // Relevant flag position is cleared
    // 相关标志位清零
    Flag_PID = 0;
    i = 0;
    j = 0;
    Data = 0;
    memset(Receive, 0,
           sizeof(unsigned char) * 50); // Clear the array to zero//数组清零
  }
  if (RC_Velocity < 0)
    RC_Velocity = 0;

  return 0;
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
int usart3_receive_data(void) {
  static unsigned char Count = 0;

  HAL_UART_Receive_IT(&huart3, &Usart3_Receive, 1);
  if (Time_count < CONTROL_DELAY)
    // Data is not processed until 10 seconds after startup
    // 开机10秒前不处理数据
    return 0;

  // Fill the array with serial data
  // 串口数据填入数组
  Receive_Data.buffer[Count] = Usart3_Receive;

  // Ensure that the first data in the array is FRAME_HEADER
  // 确保数组第一个数据为FRAME_HEADER
  if (Usart3_Receive == FRAME_HEADER || Count > 0)
    Count++;
  else
    Count = 0;

  if (Count == 11) // Verify the length of the packet //验证数据包的长度
  {
    Count = 0; // Prepare for the serial port data to be refill into the array
               // //为串口数据重新填入数组做准备
    if (Receive_Data.buffer[10] ==
        FRAME_TAIL) // Verify the frame tail of the packet //验证数据包的帧尾
    {
      // Data exclusionary or bit check calculation, mode 0 is sent data check
      // 数据异或位校验计算，模式0是发送数据校验
      if (Receive_Data.buffer[9] == Check_Sum(9, 0)) {
        float Vz;
        // All modes flag position 0, USART3 control mode
        // 所有模式标志位置0，为Usart3控制模式
        PS2_ON_Flag = 0;
        Remote_ON_Flag = 0;
        APP_ON_Flag = 0;
        CAN_ON_Flag = 0;
        Usart1_ON_Flag = 0;
        Usart5_ON_Flag = 0;
        command_lost_count = 0;
        // Calculate the target speed of three axis from serial data, unit m/s
        // 从串口数据求三轴目标速度， 单位m/s
        Move_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3],
                                             Receive_Data.buffer[4]);
        Move_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5],
                                             Receive_Data.buffer[6]);
        Vz = XYZ_Target_Speed_transition(Receive_Data.buffer[7],
                                         Receive_Data.buffer[8]);
        if (Car_Mode == Akm_Car) {
          Move_Z = Vz_to_Akm_Angle(Move_X, Vz);
        } else {
          Move_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7],
                                               Receive_Data.buffer[8]);
        }
      }
    }
  }

  return 0;
}

/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
函数功能：串口5接收中断
入口参数：无
返回  值：无
**************************************************************************/
int usart5_receive_data(void) {
  static unsigned char Count = 0;

  HAL_UART_Receive_IT(&huart5, &Usart5_Receive, 1);
  if (Time_count < CONTROL_DELAY)
    // Data is not processed until 10 seconds after startup
    // 开机10秒前不处理数据
    return 0;

  // Fill the array with serial data
  // 串口数据填入数组
  Receive_Data.buffer[Count] = Usart5_Receive;

  // Ensure that the first data in the array is FRAME_HEADER
  // 确保数组第一个数据为FRAME_HEADER
  if (Usart5_Receive == FRAME_HEADER || Count > 0)
    Count++;
  else
    Count = 0;

  if (Count == 11) // Verify the length of the packet //验证数据包的长度
  {
    Count = 0; // Prepare for the serial port data to be refill into the array
               // //为串口数据重新填入数组做准备
    if (Receive_Data.buffer[10] ==
        FRAME_TAIL) // Verify the frame tail of the packet //验证数据包的帧尾
    {
      // Data exclusionary or bit check calculation, mode 0 is sent data check
      // 数据异或位校验计算，模式0是发送数据校验
      if (Receive_Data.buffer[9] == Check_Sum(9, 0)) {
        float Vz;
        // All modes flag position 0, USART3 control mode
        // 所有模式标志位置0，为Usart5控制模式
        PS2_ON_Flag = 0;
        Remote_ON_Flag = 0;
        APP_ON_Flag = 0;
        CAN_ON_Flag = 0;
        Usart5_ON_Flag = 0;
        command_lost_count = 0;
        // Calculate the target speed of three axis from serial data, unit m/s
        // 从串口数据求三轴目标速度， 单位m/s
        Move_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3],
                                             Receive_Data.buffer[4]);
        Move_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5],
                                             Receive_Data.buffer[6]);
        Vz = XYZ_Target_Speed_transition(Receive_Data.buffer[7],
                                         Receive_Data.buffer[8]);
        if (Car_Mode == Akm_Car) {
          Move_Z = Vz_to_Akm_Angle(Move_X, Vz);
        } else {
          Move_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7],
                                               Receive_Data.buffer[8]);
        }
      }
    }
  }

  return 0;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type
data, the unit reduction is converted Input   : 8 bits high, 8 bits low Output
: The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来目标前进速度Vx、目标角速度Vz，转换为阿克曼小车的右前轮转角
入口参数：目标前进速度Vx、目标角速度Vz，单位：m/s，rad/s
返回  值：阿克曼小车的右前轮转角，单位：rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz) {
  float R, AngleR, Min_Turn_Radius;
  // float AngleL;

  // Ackermann car needs to set minimum turning radius
  // If the target speed requires a turn radius less than the minimum turn
  // radius, This will greatly improve the friction force of the car, which will
  // seriously affect the control effect 阿克曼小车需要设置最小转弯半径
  // 如果目标速度要求的转弯半径小于最小转弯半径，
  // 会导致小车运动摩擦力大大提高，严重影响控制效果
  Min_Turn_Radius = MINI_AKM_MIN_TURN_RADIUS;

  if (Vz != 0 && Vx != 0) {
    // If the target speed requires a turn radius less than the minimum turn
    // radius 如果目标速度要求的转弯半径小于最小转弯半径
    if (float_abs(Vx / Vz) <= Min_Turn_Radius) {
      // Reduce the target angular velocity and increase the turning radius to
      // the minimum turning radius in conjunction with the forward speed
      // 降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
      if (Vz > 0)
        Vz = float_abs(Vx) / (Min_Turn_Radius);
      else
        Vz = -float_abs(Vx) / (Min_Turn_Radius);
    }
    R = Vx / Vz;
    // AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
    AngleR = atan(Axle_spacing / (R + 0.5f * Wheel_spacing));
  } else {
    AngleR = 0;
  }

  return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type
data, the unit reduction is converted Input   : 8 bits high, 8 bits low Output
: The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(unsigned char High, unsigned char Low) {
  // Data conversion intermediate variable
  // 数据转换的中间变量
  short transition;

  // 将高8位和低8位整合成一个16位的short型数据
  // The high 8 and low 8 bits are integrated into a 16-bit short data
  transition = ((High << 8) + Low);
  return transition / 1000 +
         (transition % 1000) *
             0.001; // Unit conversion, mm/s->m/s //单位转换, mm/s->m/s
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the
received data, 1-Validate the sent data Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
unsigned char Check_Sum(unsigned char Count_Number, unsigned char Mode) {
  unsigned char check_sum = 0, k;

  // Validate the data to be sent
  // 对要发送的数据进行校验
  if (Mode == 1)
    for (k = 0; k < Count_Number; k++) {
      check_sum = check_sum ^ Send_Data.buffer[k];
    }

  // Verify the data received
  // 对接收到的数据进行校验
  if (Mode == 0)
    for (k = 0; k < Count_Number; k++) {
      check_sum = check_sum ^ Receive_Data.buffer[k];
    }
  return check_sum;
}
