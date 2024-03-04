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
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void data_transition(void) {
  Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; // Frame_header //֡ͷ
  Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     // Frame_tail //֡β

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

  // The acceleration of the triaxial acceleration //���ٶȼ�������ٶ�
  Send_Data.Sensor_Str.Accelerometer.X_data =
      accel[1]; // The accelerometer Y-axis is converted to the ros coordinate X
                // axis //���ٶȼ�Y��ת����ROS����X��
  Send_Data.Sensor_Str.Accelerometer.Y_data =
      -accel[0]; // The accelerometer X-axis is converted to the ros coordinate
                 // y axis //���ٶȼ�X��ת����ROS����Y��
  Send_Data.Sensor_Str.Accelerometer.Z_data =
      accel[2]; // The accelerometer Z-axis is converted to the ros coordinate Z
                // axis //���ٶȼ�Z��ת����ROS����Z��

  // The Angle velocity of the triaxial velocity //���ٶȼ�������ٶ�
  Send_Data.Sensor_Str.Gyroscope.X_data =
      gyro[1]; // The Y-axis is converted to the ros coordinate X axis
               // //���ٶȼ�Y��ת����ROS����X��
  Send_Data.Sensor_Str.Gyroscope.Y_data =
      -gyro[0]; // The X-axis is converted to the ros coordinate y axis
                // //���ٶȼ�X��ת����ROS����Y��
  if (Flag_Stop == 0)
    // If the motor control bit makes energy state, the z-axis velocity is sent
    // normall
    // ����������λʹ��״̬����ô��������Z����ٶ�
    Send_Data.Sensor_Str.Gyroscope.Z_data = gyro[2];
  else
    // If the robot is static (motor control dislocation), the z-axis is 0
    // ����������Ǿ�ֹ�ģ��������λʧ�ܣ�����ô���͵�Z����ٶ�Ϊ0
    Send_Data.Sensor_Str.Gyroscope.Z_data = 0;

  // Battery voltage (this is a thousand times larger floating point number,
  // which will be reduced by a thousand times as well as receiving the data).
  // ��ص�ѹ(���ｫ�������Ŵ�һǧ�����䣬��Ӧ���ڽ��ն��ڽ��յ����ݺ�Ҳ����Сһǧ��)
  Send_Data.Sensor_Str.Power_Voltage = (short)(get_battery_voltage() * 1000);
  Send_Data.Sensor_Str.CCD1 = (short)(get_CCD1_voltage() * 1000);
  Send_Data.Sensor_Str.CCD2 = (short)(get_CCD2_voltage() * 1000);

  Send_Data.buffer[0] = Send_Data.Sensor_Str.Frame_Header; // Frame_heade //֡ͷ
  Send_Data.buffer[1] =
      Flag_Stop; // Car software loss marker //С�����ʧ�ܱ�־λ

  // The three-axis speed of / / car is split into two eight digit Numbers
  // С�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
  Send_Data.buffer[2] = Send_Data.Sensor_Str.X_speed >> 8;
  Send_Data.buffer[3] = Send_Data.Sensor_Str.X_speed;
  Send_Data.buffer[4] = Send_Data.Sensor_Str.Y_speed >> 8;
  Send_Data.buffer[5] = Send_Data.Sensor_Str.Y_speed;
  Send_Data.buffer[6] = Send_Data.Sensor_Str.Z_speed >> 8;
  Send_Data.buffer[7] = Send_Data.Sensor_Str.Z_speed;

  // The acceleration of the triaxial axis of / / imu accelerometer is divided
  // into two eight digit reams
  // IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
  Send_Data.buffer[8] = Send_Data.Sensor_Str.Accelerometer.X_data >> 8;
  Send_Data.buffer[9] = Send_Data.Sensor_Str.Accelerometer.X_data;
  Send_Data.buffer[10] = Send_Data.Sensor_Str.Accelerometer.Y_data >> 8;
  Send_Data.buffer[11] = Send_Data.Sensor_Str.Accelerometer.Y_data;
  Send_Data.buffer[12] = Send_Data.Sensor_Str.Accelerometer.Z_data >> 8;
  Send_Data.buffer[13] = Send_Data.Sensor_Str.Accelerometer.Z_data;

  // The axis of the triaxial velocity of the / /imu is divided into two eight
  // digits IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
  Send_Data.buffer[14] = Send_Data.Sensor_Str.Gyroscope.X_data >> 8;
  Send_Data.buffer[15] = Send_Data.Sensor_Str.Gyroscope.X_data;
  Send_Data.buffer[16] = Send_Data.Sensor_Str.Gyroscope.Y_data >> 8;
  Send_Data.buffer[17] = Send_Data.Sensor_Str.Gyroscope.Y_data;
  Send_Data.buffer[18] = Send_Data.Sensor_Str.Gyroscope.Z_data >> 8;
  Send_Data.buffer[19] = Send_Data.Sensor_Str.Gyroscope.Z_data;

  // Battery voltage, split into two 8 digit Numbers
  // ��ص�ѹ,���Ϊ����8λ���ݷ���
  Send_Data.buffer[20] = Send_Data.Sensor_Str.Power_Voltage >> 8;
  Send_Data.buffer[21] = Send_Data.Sensor_Str.Power_Voltage;
  Send_Data.buffer[22] = Send_Data.Sensor_Str.CCD1 >> 8;
  Send_Data.buffer[23] = Send_Data.Sensor_Str.CCD1;
  Send_Data.buffer[24] = Send_Data.Sensor_Str.CCD2 >> 8;
  Send_Data.buffer[25] = Send_Data.Sensor_Str.CCD2;

  // Data check digit calculation, Pattern 1 is a data check
  // ����У��λ���㣬ģʽ1�Ƿ�������У��
  Send_Data.buffer[26] = Check_Sum(26, 1);

  Send_Data.buffer[27] = Send_Data.Sensor_Str.Frame_Tail; // Frame_tail //֡β
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
�������ܣ�����1��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART1_SEND(void) {
  HAL_UART_Transmit(&huart1, Send_Data.buffer, SEND_DATA_SIZE, HAL_MAX_DELAY);
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
�������ܣ�����3��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART3_SEND(void) {
  HAL_UART_Transmit(&huart3, Send_Data.buffer, SEND_DATA_SIZE, HAL_MAX_DELAY);
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
�������ܣ�����5��������
��ڲ�������
����  ֵ����
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
    // ����10��ǰ����������
    return 0;

  // Fill the array with serial data
  // ����������������
  rxbuf[Count] = Usart1_Receive;

  // Ensure that the first data in the array is FRAME_HEADER
  // ȷ�������һ������ΪFRAME_HEADER
  if (Usart1_Receive == FRAME_HEADER || Count > 0)
    Count++;
  else
    Count = 0;

  if (Count == 11) // Verify the length of the packet //��֤���ݰ��ĳ���
  {
    Count = 0; // Prepare for the serial port data to be refill into the array
               // //Ϊ����������������������׼��
    if (rxbuf[10] ==
        FRAME_TAIL) // Verify the frame tail of the packet //��֤���ݰ���֡β
    {
      for (i = 0; i < 9; i++) {
        // XOR bit check, used to detect data error
        // ���λУ�飬���ڼ�������Ƿ����
        check = rxbuf[i] ^ check;
      }
      if (check == rxbuf[9])
        // XOR bit check successful
        // ���λУ��ɹ�
        error = 0;

      if (error == 0) {
        float Vz;
        if (Usart1_ON_Flag == 0) {
          // Serial port 1 controls flag position 1, other flag position 0
          // ����1���Ʊ�־λ��1��������־λ��0
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
        // �Ӵ�������������Ŀ���ٶȣ��ָ�8λ�͵�8λ ��λmm/s
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
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int usart2_receive_data(void) {

  static unsigned char Flag_PID, i, j, Receive[50], Last_Usart2_Receive;
  static float Data;

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&Usart2_Receive, 4);

  if (Deviation_Count < CONTROL_DELAY)
    // Data is not processed until 10 seconds after startup
    // ����10��ǰ����������
    return 0;

  if (Usart2_Receive == 0x41 && Last_Usart2_Receive == 0x41 && APP_ON_Flag == 0)
    // 10 seconds after startup, press the forward button of APP to enter APP
    // control mode
    // The APP controls the flag position 1 and the other flag position 0
    // ����10��֮�󣬰���APP��ǰ��������APP����ģʽ
    // APP���Ʊ�־λ��1��������־λ��0
    PS2_ON_Flag = 0, Remote_ON_Flag = 0, APP_ON_Flag = 1, CAN_ON_Flag = 0,
    Usart1_ON_Flag = 0, Usart5_ON_Flag = 0;
  Last_Usart2_Receive = Usart2_Receive;

  if (Usart2_Receive == 0x4B)
    // Enter the APP steering control interface
    // ����APPת����ƽ���
    Turn_Flag = 1;
  else if (Usart2_Receive == 0x49 || Usart2_Receive == 0x4A)
    // Enter the APP direction control interface
    // ����APP������ƽ���
    Turn_Flag = 0;

  if (Turn_Flag == 0) {
    // App rocker control interface command
    // APPҡ�˿��ƽ�������
    if (Usart2_Receive >= 0x41 && Usart2_Receive <= 0x48) {
      Flag_Direction = Usart2_Receive - 0x40;
    } else if (Usart2_Receive <= 8) {
      Flag_Direction = Usart2_Receive;
    } else
      Flag_Direction = 0;
  } else if (Turn_Flag == 1) {
    // APP steering control interface command
    // APPת����ƽ�������
    if (Usart2_Receive == 0x43)
      Flag_Left = 0, Flag_Right = 1; // Right rotation //����ת
    else if (Usart2_Receive == 0x47)
      Flag_Left = 1, Flag_Right = 0; // Left rotation  //����ת
    else
      Flag_Left = 0, Flag_Right = 0;
    if (Usart2_Receive == 0x41 || Usart2_Receive == 0x45)
      Flag_Direction = Usart2_Receive - 0x40;
    else
      Flag_Direction = 0;
  }
  if (Usart2_Receive == 0x58)
    RC_Velocity =
        RC_Velocity + 100; // Accelerate the keys, +100mm/s //���ٰ�����+100mm/s
  if (Usart2_Receive == 0x59)
    RC_Velocity =
        RC_Velocity - 100; // Slow down buttons,   -100mm/s //���ٰ�����-100mm/s

  // The following is the communication with the APP debugging interface
  // ��������APP���Խ���ͨѶ
  if (Usart2_Receive == 0x7B)
    Flag_PID = 1; // The start bit of the APP parameter instruction
                  // //APP����ָ����ʼλ
  if (Usart2_Receive == 0x7D)
    Flag_PID =
        2; // The APP parameter instruction stops the bit //APP����ָ��ֹͣλ

  if (Flag_PID == 1) // Collect data //�ɼ�����
  {
    Receive[i] = Usart2_Receive;
    i++;
  }
  if (Flag_PID == 2) // Analyze the data //��������
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
    // ��ر�־λ����
    Flag_PID = 0;
    i = 0;
    j = 0;
    Data = 0;
    memset(Receive, 0,
           sizeof(unsigned char) * 50); // Clear the array to zero//��������
  }
  if (RC_Velocity < 0)
    RC_Velocity = 0;

  return 0;
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int usart3_receive_data(void) {
  static unsigned char Count = 0;

  HAL_UART_Receive_IT(&huart3, &Usart3_Receive, 1);
  if (Time_count < CONTROL_DELAY)
    // Data is not processed until 10 seconds after startup
    // ����10��ǰ����������
    return 0;

  // Fill the array with serial data
  // ����������������
  Receive_Data.buffer[Count] = Usart3_Receive;

  // Ensure that the first data in the array is FRAME_HEADER
  // ȷ�������һ������ΪFRAME_HEADER
  if (Usart3_Receive == FRAME_HEADER || Count > 0)
    Count++;
  else
    Count = 0;

  if (Count == 11) // Verify the length of the packet //��֤���ݰ��ĳ���
  {
    Count = 0; // Prepare for the serial port data to be refill into the array
               // //Ϊ����������������������׼��
    if (Receive_Data.buffer[10] ==
        FRAME_TAIL) // Verify the frame tail of the packet //��֤���ݰ���֡β
    {
      // Data exclusionary or bit check calculation, mode 0 is sent data check
      // �������λУ����㣬ģʽ0�Ƿ�������У��
      if (Receive_Data.buffer[9] == Check_Sum(9, 0)) {
        float Vz;
        // All modes flag position 0, USART3 control mode
        // ����ģʽ��־λ��0��ΪUsart3����ģʽ
        PS2_ON_Flag = 0;
        Remote_ON_Flag = 0;
        APP_ON_Flag = 0;
        CAN_ON_Flag = 0;
        Usart1_ON_Flag = 0;
        Usart5_ON_Flag = 0;
        command_lost_count = 0;
        // Calculate the target speed of three axis from serial data, unit m/s
        // �Ӵ�������������Ŀ���ٶȣ� ��λm/s
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
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int usart5_receive_data(void) {
  static unsigned char Count = 0;

  HAL_UART_Receive_IT(&huart5, &Usart5_Receive, 1);
  if (Time_count < CONTROL_DELAY)
    // Data is not processed until 10 seconds after startup
    // ����10��ǰ����������
    return 0;

  // Fill the array with serial data
  // ����������������
  Receive_Data.buffer[Count] = Usart5_Receive;

  // Ensure that the first data in the array is FRAME_HEADER
  // ȷ�������һ������ΪFRAME_HEADER
  if (Usart5_Receive == FRAME_HEADER || Count > 0)
    Count++;
  else
    Count = 0;

  if (Count == 11) // Verify the length of the packet //��֤���ݰ��ĳ���
  {
    Count = 0; // Prepare for the serial port data to be refill into the array
               // //Ϊ����������������������׼��
    if (Receive_Data.buffer[10] ==
        FRAME_TAIL) // Verify the frame tail of the packet //��֤���ݰ���֡β
    {
      // Data exclusionary or bit check calculation, mode 0 is sent data check
      // �������λУ����㣬ģʽ0�Ƿ�������У��
      if (Receive_Data.buffer[9] == Check_Sum(9, 0)) {
        float Vz;
        // All modes flag position 0, USART3 control mode
        // ����ģʽ��־λ��0��ΪUsart5����ģʽ
        PS2_ON_Flag = 0;
        Remote_ON_Flag = 0;
        APP_ON_Flag = 0;
        CAN_ON_Flag = 0;
        Usart5_ON_Flag = 0;
        command_lost_count = 0;
        // Calculate the target speed of three axis from serial data, unit m/s
        // �Ӵ�������������Ŀ���ٶȣ� ��λm/s
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
�������ܣ�����λ��������Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz��ת��Ϊ������С������ǰ��ת��
��ڲ�����Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz����λ��m/s��rad/s
����  ֵ��������С������ǰ��ת�ǣ���λ��rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz) {
  float R, AngleR, Min_Turn_Radius;
  // float AngleL;

  // Ackermann car needs to set minimum turning radius
  // If the target speed requires a turn radius less than the minimum turn
  // radius, This will greatly improve the friction force of the car, which will
  // seriously affect the control effect ������С����Ҫ������Сת��뾶
  // ���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶��
  // �ᵼ��С���˶�Ħ���������ߣ�����Ӱ�����Ч��
  Min_Turn_Radius = MINI_AKM_MIN_TURN_RADIUS;

  if (Vz != 0 && Vx != 0) {
    // If the target speed requires a turn radius less than the minimum turn
    // radius ���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶
    if (float_abs(Vx / Vz) <= Min_Turn_Radius) {
      // Reduce the target angular velocity and increase the turning radius to
      // the minimum turning radius in conjunction with the forward speed
      // ����Ŀ����ٶȣ����ǰ���ٶȣ����ת��뾶����Сת��뾶
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
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(unsigned char High, unsigned char Low) {
  // Data conversion intermediate variable
  // ����ת�����м����
  short transition;

  // ����8λ�͵�8λ���ϳ�һ��16λ��short������
  // The high 8 and low 8 bits are integrated into a 16-bit short data
  transition = ((High << 8) + Low);
  return transition / 1000 +
         (transition % 1000) *
             0.001; // Unit conversion, mm/s->m/s //��λת��, mm/s->m/s
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the
received data, 1-Validate the sent data Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
unsigned char Check_Sum(unsigned char Count_Number, unsigned char Mode) {
  unsigned char check_sum = 0, k;

  // Validate the data to be sent
  // ��Ҫ���͵����ݽ���У��
  if (Mode == 1)
    for (k = 0; k < Count_Number; k++) {
      check_sum = check_sum ^ Send_Data.buffer[k];
    }

  // Verify the data received
  // �Խ��յ������ݽ���У��
  if (Mode == 0)
    for (k = 0; k < Count_Number; k++) {
      check_sum = check_sum ^ Receive_Data.buffer[k];
    }
  return check_sum;
}
