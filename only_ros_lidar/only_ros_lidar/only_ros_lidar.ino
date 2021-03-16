#define X   A0
#define Y   A1

#define LENGTH_BET_WHEEL    0.281     // 28.1cm = 0.281m
#define WHEEL_RADIUS        0.0625    // 6.25cm = 0.0625m 

#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ros_lib/laser_values/caselidar.h>

//int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };


char joy_flag = 0;         // 모바일로봇 run flag
int lmotor_speed = 0;
int rmotor_speed = 0;

float error;
float dt = 0.000;

// PID variable
float desire_range = 50;
float front_prev_range = 0;
float front_I_control = 0;
float front_kp = 20;
float front_ki = 0;
float front_kd = 0;
float front_out = 0;


float left_prev_range = 0;
float left_I_control = 0;
float left_kp = 20;
float left_ki = 0;
float left_kd = 0;
float left_out = 0;


float right_prev_range = 0;
float right_I_control = 0;
float right_kp = 20;
float right_ki = 0;
float right_kd = 0;
float right_out = 0;

String MODE;
unsigned char mode_num = 0;
float front_min_range = 0;
float left_min_range = 0;
float right_min_range = 0;

int x = 0;
int y = 0;

ros::NodeHandle nh;


void Joystick_Value_Normal(int x, int y, int* lmotor_speed, int* rmotor_speed);
//void Joystick_Value_Stop(int x, int y, int* lmotor_speed, int* rmotor_speed, float* front_out);
//void Joystick_Value_Left(int x, int y, int* lmotor_speed, int* rmotor_speed, float* left_out);
//void Joystick_Value_Right(int x, int y, int* lmotor_speed, int* rmotor_speed, float* right_out);
//void Joystick_Control(int* lmotor_speed, int* rmotor_speed);
unsigned short CRC16(char *puchMsg, int usDataLen);
void Modbus_Table(char id, char func, char addrH, char addrL, int data);


// PID함수
void StdPID(float *desire_range, float *current_range, float *prev_range, float *I_control, float *Kp, float *Ki, float *Kd, float *PID_control)
{
    
    float dInput;
    float P_control;
    float D_control;
    

    error = *desire_range - *current_range;        // 에러값 
    dInput = *current_range - *prev_range;
    *prev_range = *current_range;
   
    P_control = (*Kp) * error;
    *I_control += (*Ki) * (error * dt);
    D_control = -(*Kd) * (dInput/ dt);

    
    *PID_control = P_control + *I_control + D_control;  // PID 출력 
    
    //Serial.println(*PID_control);   
}



void CalcMotorPID()
{
    StdPID(&desire_range, &front_min_range, &front_prev_range, &front_I_control, &front_kp, &front_ki, &front_kd, &front_out);
    StdPID(&desire_range, &left_min_range, &left_prev_range, &left_I_control, &left_kp, &left_ki, &left_kd, &left_out);
    StdPID(&desire_range, &right_min_range, &right_prev_range, &right_I_control, &right_kp, &right_ki, &right_kd, &right_out);
    
}



void Take_action(unsigned char mode_num)
{
  switch(mode_num)
  {
    case 0:
      CalcMotorPID();
      Joystick_Value_Normal(x,y,&lmotor_speed, &rmotor_speed);
      Joystick_Control(&lmotor_speed, &rmotor_speed);
      delay(2);
      break;
    case 1:
      CalcMotorPID();
      Joystick_Value_Stop(x,y,&lmotor_speed, &rmotor_speed, &front_out);
      Joystick_Control(&lmotor_speed, &rmotor_speed);
      delay(2);
      break;
    case 2:
      CalcMotorPID();
      Joystick_Value_Left(x,y,&lmotor_speed, &rmotor_speed, &left_out);
      Joystick_Control(&lmotor_speed, &rmotor_speed);
      delay(2);
      break;
    case 3:
      CalcMotorPID();
      Joystick_Value_Stop(x,y,&lmotor_speed, &rmotor_speed, &right_out);
      Joystick_Control(&lmotor_speed, &rmotor_speed);
      delay(2);
      break;
    default:
      break;
    
  }
}
void calb_lidar(const laser_values::caselidar& msg)
{
  MODE = msg.object_pose;
  front_min_range = msg.front_range;   // meter
  left_min_range = msg.left_range;     // meter
  right_min_range = msg.right_range;   // meter

  front_min_range *= 100;               // cm
  left_min_range *= 100;                // cm
  right_min_range *= 100;               // cm

  if(MODE == "normal")
  {
    mode_num = 0;
  }
  else if(MODE == "stop")
  {
    mode_num = 1;
    
  }
  else if(MODE == "left")
  {
    mode_num = 2;
    
  }
  else if(MODE == "right")
  {
    mode_num = 3;
   
  }

  Take_action(mode_num);  
}


ros::Subscriber<laser_values::caselidar>lidar_data("lidar_ranges",calb_lidar);


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  Serial1.begin(115200);
  analogReadResolution(12);

  nh.initNode();
  nh.subscribe(lidar_data);  
}


void loop() {
  
  x=analogRead(X);
  y=analogRead(Y);
  
  nh.spinOnce();
  delay(1);


}

void Joystick_Value_Normal(int x, int y, int* lmotor_speed, int* rmotor_speed)
{
  int j_x = x-2050; 
  int j_y = y-2055;

  *lmotor_speed = -(j_y + 2.2*((j_x*LENGTH_BET_WHEEL/4))); 
  *rmotor_speed = j_y - 2.2*((j_x*LENGTH_BET_WHEEL)/4);

  
}


void Joystick_Value_Stop(int x, int y, int* lmotor_speed, int* rmotor_speed, float* front_out)
{
//  int j_x = x-2050; 
//  int j_y = y-2055;
//
//  j_y -= *front_out;
//
//  *lmotor_speed = -(j_y + 2.2*((j_x*LENGTH_BET_WHEEL/4))); 
//  *rmotor_speed = j_y - 2.2*((j_x*LENGTH_BET_WHEEL)/4); 
  
}


void Joystick_Value_Left(int x, int y, int* lmotor_speed, int* rmotor_speed, float* left_out)
{
//  int j_x = x-2050; 
//  int j_y = y-2055;
//
//  j_x -= *left_out;
//  *lmotor_speed = -(j_y + 2.2*((j_x*LENGTH_BET_WHEEL/4))); 
//  *rmotor_speed = j_y - 2.2*((j_x*LENGTH_BET_WHEEL)/4); 
}


void Joystick_Value_Right(int x, int y, int* lmotor_speed, int* rmotor_speed, float* right_out)
{
//  int j_x = x-2050; 
//  int j_y = y-2055;
//
//  j_x +=*right_out;
//
//  *lmotor_speed = -(j_y + 2.2*((j_x*LENGTH_BET_WHEEL/4))); 
//  *rmotor_speed = j_y - 2.2*((j_x*LENGTH_BET_WHEEL)/4); 
}

void Joystick_Control(int* lmotor_speed, int* rmotor_speed)
{
  if(!(((*lmotor_speed>-30)&&(*lmotor_speed<30))&&((*rmotor_speed>-30)&&(*rmotor_speed<30))))
  {
    *lmotor_speed = 0.8 * (*lmotor_speed);
    *rmotor_speed = 0.8 * (*rmotor_speed);
    
    Modbus_Table(0x01,0x06,0x00,0x79,*lmotor_speed);
    delay(2);
    Modbus_Table(0x02,0x06,0x00,0x79,*rmotor_speed);
    delay(2);

  //  MD_Packet(183,172,1,252,1,1);

    if(joy_flag)
    {
      Modbus_Table(0x01,0x06,0x00,0x78,1);
      delay(2);
      Modbus_Table(0x02,0x06,0x00,0x78,1);
      delay(2);

      joy_flag = 0;
    }
  } else
  { 
    Modbus_Table(0x01,0x06,0x00,0x78,2);
    delay(20);
    Modbus_Table(0x02,0x06,0x00,0x78,2);
    delay(20);

    joy_flag = 1;
  }
}


unsigned short CRC16(char *puchMsg, int usDataLen)
{
  int i;
  unsigned short crc, flag;
  crc = 0xffff;

  while(usDataLen--){
    crc ^= *puchMsg++;

    for(i=0;i<8;i++)
    {
       flag = crc & 0x0001;
       crc >>= 1;
       if(flag) crc ^= 0xA001;
    }
  }
  return crc;
}


void Modbus_Table(char id, char func, char addrH, char addrL, int data)
{
  char protocol_buf[8] = {0,};
  unsigned char dataL = 0;
  unsigned char dataH = 0;
  unsigned short crc16 = 0;

  int i = 0;

  dataL = (0x00FF)&data;
  dataH = ((0xFF00)&data)>>8;

  protocol_buf[i++] = id;
  protocol_buf[i++] = func;
  protocol_buf[i++] = addrH;
  protocol_buf[i++] = addrL;
  protocol_buf[i++] = dataH;
  protocol_buf[i++] = dataL;

  crc16 = CRC16(protocol_buf, 6);

  protocol_buf[6] = (unsigned char)((crc16>>0) & 0x00ff);
  protocol_buf[7] = (unsigned char)((crc16>>8) & 0x00ff);

  Serial1.write(protocol_buf, 8);
}
