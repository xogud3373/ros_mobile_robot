#define X   A0
#define Y   A1

#define LENGTH_BET_WHEEL    0.281     // 28.1cm = 0.281m
#define WHEEL_RADIUS        0.0625    // 6.25cm = 0.0625m 

#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_values/caselidar.h>

char joy_flag = 0;         // 모바일로봇 run flag
int lmotor_speed = 0;
int rmotor_speed = 0;

float error;
float dt = 0.000;

// PID variable
// Front
float desire_range = 50;
float front_prev_range = 0;
float front_I_control = 0;
float front_kp = 100;
float front_ki = 0;
float front_kd = 0;
float front_out = 0;

// Left
float left_prev_range = 0;
float left_I_control = 0;
float left_kp = 100;
float left_ki = 0;
float left_kd = 0;
float left_out = 0;

float left_x_kp = 40;
float left_x_ki = 0;
float left_x_kd = 0;
float left_x_low = 0;

// Right
float right_prev_range = 0;
float right_I_control = 0;
float right_kp = 100;
float right_ki = 0;
float right_kd = 0;
float right_out = 0;

float right_x_kp = 40;
float right_x_ki = 0;
float right_x_kd = 0;
float right_x_low = 0;

// Custom message variable
String object_loc;
unsigned char mode_num = 0;
float front_min_range = 0;
float left_min_range = 0;
float right_min_range = 0;

// joystick variable
int x = 0;
int y = 0;

ros::NodeHandle nh;


void Read_Joystick_Value(int* x, int* y);
void PID_Joystick_Value(int* x, int* y, int* lmotor_speed, int* rmotor_speed, float* front_out, float* left_out, float* right_out, float* left_x_low, float* right_x_low);
void Joystick_Control(int* lmotor_speed, int* rmotor_speed);
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
    //D_control = -(*Kd) * (dInput/ dt);

    
    *PID_control = P_control + (*I_control) + D_control;  // PID 출력

      
}


// PID 게인 결정 함수
void CalcMotorPID()
{
    StdPID(&desire_range, &front_min_range, &front_prev_range, &front_I_control, &front_kp, &front_ki, &front_kd, &front_out);
    StdPID(&desire_range, &left_min_range, &left_prev_range, &left_I_control, &left_kp, &left_ki, &left_kd, &left_out);
    StdPID(&desire_range, &right_min_range, &right_prev_range, &right_I_control, &right_kp, &right_ki, &right_kd, &right_out);    
    StdPID(&desire_range, &left_min_range, &left_prev_range, &left_I_control, &left_x_kp, &left_x_ki, &left_x_kd, &left_x_low);    
    StdPID(&desire_range, &right_min_range, &right_prev_range, &right_I_control, &right_x_kp, &right_x_ki, &right_x_kd, &right_x_low);    

}


/*  @brief  : 장애물 위치에 따른 조이스틱 값 제어 함수
 *  @explan : 조이스틱의 값을 아날로그로 받는다. 
 *          : 장애물 방향에 따른 모드에 따라 라이다 거리값을 pid하여 조이스틱 각속도를 없애준다. 
 *          : PID_Joystick_Value함수로 나온 값을 모드버스 프로토콜 속도로 변경하여 모바일로봇 제어
 *          
 *  @param  : front_out : 전방장애물 발견 시 라이다와 장애물 간의 거리값을 pid한 값 
 *          : left_out : 오른쪽 장애물 발견 시 라이다와 장애물 간의 거리값을 pid한 값
 *          : right_out : 왼쪽 장애물 발견 시 라이다와 장애물 간의 거리값을 pid한 
*/

void Take_action(unsigned char mode_num)
{
  Read_Joystick_Value(&x, &y);
  CalcMotorPID();
  PID_Joystick_Value(&x,&y,&lmotor_speed, &rmotor_speed, &front_out, &left_out, &right_out, &left_x_low, &right_x_low);
  Joystick_Control(&lmotor_speed, &rmotor_speed);
  
}

/*  @brief : custom message를 subscribe  
 *  @explain : msg.object_pose : String형식 -> 어느 방향에 장애물이 있는지
 *           : msg.front_range : Float32형식 -> 전방 장애물과 로봇간의 최소거리
 *           : msg.left_range  : Float32형식 -> 왼쪽, 왼쪽 대각선 장애물과 로봇간의 최소거리
 *           : msg.right_range : Float32형식 -> 오른쪽, 오른쪽 대각선 장애물과 로봇간의 최소거리
 */
void calb_lidar(const laser_values::caselidar& msg)
{
  
  object_loc = msg.object_pose;
  front_min_range = msg.front_range;   // meter
  left_min_range = msg.left_range;     // meter
  right_min_range = msg.right_range;   // meter

  front_min_range *= 100.0;               // meter to cm
  left_min_range *= 100.0;                // meter to cm
  right_min_range *= 100.0;               // meter to cm

  if(object_loc == "normal")                  // 장애물 발견 안될 때
  {
    mode_num = 0;
  }
  else if(object_loc == "stop")               // 전방 및 모든 방향에 장애물 발견 시
  {
    mode_num = 1;
    
  }
  else if(object_loc == "right")               // 오른쪽 장애물 발견 시
  {
    mode_num = 2;
    
  }
  else if(object_loc == "left")              // 왼쪽 장애물 발견 
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


void loop() 
{
  
  nh.spinOnce();



}

// 조이스틱 값 아날로그로 받는 함수
void Read_Joystick_Value(int *x, int *y)
{
  *x=analogRead(X);
  *y=analogRead(Y);
}

// 모드에 따른 조이스틱 값 제어 함수
void PID_Joystick_Value(int *x, int *y, int* lmotor_speed, int* rmotor_speed, float* front_out, float* left_out, float* right_out, float* left_x_low, float* right_x_low)
{
  int joy_x = *x-2050; 
  int joy_y = *y-2055;

  //int j_x = 0;
  
  switch(mode_num)
  {
    case 0:                                 // normal
      break;
    case 1:                                 // stop
      joy_y -= int((*front_out));
      break;
    case 2:                                 // turn left
      joy_x -= int((*right_out));
      joy_y -= int((*right_x_low));

      break;
    case 3:                                 // turn right
      joy_x += int((*left_out));
      joy_y -= int((*left_x_low));
      break;
    default:
      break;
    
  }



  Serial.println("-----------");
  Serial.print("front_out : ");
  Serial.println(*front_out);
  
  Serial.print("left_out : ");
  Serial.println(*left_out);
  
  Serial.print("right_out : ");
  Serial.println(*right_out);
//
//  Serial.print("left_x_low : ");
//  Serial.println(*left_x_low);
//  
//  Serial.print("right_x_low : ");
//  Serial.println(*right_x_low);
//  
//  
  Serial.println("-----------");
  Serial.print("joy_x : ");
  Serial.println(joy_x);
  
  Serial.print("joy_y : ");
  Serial.println(joy_y);
  
  *lmotor_speed = -(joy_y + 2.2*((joy_x*LENGTH_BET_WHEEL/4))); 
  *rmotor_speed = joy_y - 2.2*((joy_x*LENGTH_BET_WHEEL)/4);

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
  }
  else
  { 
    Modbus_Table(0x01,0x06,0x00,0x78,2);
    delay(2);
    Modbus_Table(0x02,0x06,0x00,0x78,2);
    delay(2);

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
