#define X   A0
#define Y   A1

#define LENGTH_BET_WHEEL    0.281     // 28.1cm = 0.281m
#define WHEEL_RADIUS        0.0625    // 6.25cm = 0.0625m 

#include <ros.h>
#include <geometry_msgs/Twist.h>


char joy_flag = 0;         // 모바일로봇 run flag
int lmotor_speed = 0;
int rmotor_speed = 0;

unsigned char sw1_on;
unsigned char sw2_on = 0;
double vx = 0;
double vth = 0;

ros::NodeHandle nh;

void callback(const geometry_msgs::Twist& msg)
{
  vx = msg.linear.x;
  vth = msg.angular.z;

  Autonomous_Drive(vx, vth);
}

ros::Subscriber<geometry_msgs::Twist>moblie_vel("/cmd_vel",callback);


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); 
  analogReadResolution(12);

    
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);

  nh.initNode();
  nh.subscribe(moblie_vel);  
}

void loop() {
  int x,y;
  x=analogRead(X);
  y=analogRead(Y);
  
  check_button_num();
  if(sw1_on)
  {
    nh.spinOnce();
  }
  else
  {
    Joystick_Value(x,y,&lmotor_speed,&rmotor_speed);
    Joystick_Control(&lmotor_speed,&rmotor_speed);
    delay(2);
  }

  Serial.print("sw1 : ");
  Serial.println(sw1_on);
  
}

void check_button_num()
{
  if(digitalRead(BDPIN_DIP_SW_1) == HIGH)
  {
    sw1_on = 1;
  }
  else
  {
    sw1_on = 0; 
  }
  
}


float Unit_Convert(float input, int unit)
{   
    float output = 0;         
    
    switch(unit)
    {
        case 1 :
        //degree to radian
        output = (input)*(0.0055556)*PI;          
        break;        
        
        case 2 :
        //radian to degree
        output = ((input)*180)/PI;
        break;      
        
        case 3 :
        //RPM to m/s
        output = WHEEL_RADIUS*((2*PI)/60)*(input);
        break;      
        
        case 4 :
        //m/s to RPM
        output = ((input)*60)/(2*PI*WHEEL_RADIUS);
        break;        
                                       
        case 5 :
        //RPM to speed data
        output = (input)/0.0499;
        break;
    }   
    return output;            
}


void Autonomous_Drive(float velocity, float angular_velocity)
{
  float v = 0;
  float av = 0;
  float vl_ms = 0;     // ms : m/s
  float vr_ms = 0;
  int vl_rpm = 0;
  int vr_rpm = 0;
  int vl_sd = 0;       // sd : speed data
  int vr_sd = 0;  
  
  v = velocity;             // 속도의 단위는 m/s
  av = angular_velocity;    // 각속도의 단위는 rad/s
  vl_ms = v;
  vr_ms = v;

  vl_ms = -(vl_ms + (av*LENGTH_BET_WHEEL/4));
  vr_ms = vr_ms - (av*LENGTH_BET_WHEEL/4);

  vl_rpm = Unit_Convert(vl_ms, 4);
  vr_rpm = Unit_Convert(vr_ms, 4);
  vl_sd = Unit_Convert(vl_rpm, 5);
  vr_sd = Unit_Convert(vr_rpm, 5);

  Modbus_Table(0x01,0x06,0x00,0x79,vl_sd);
  delay(2);
  Modbus_Table(0x02,0x06,0x00,0x79,vr_sd);
  delay(2);
  Modbus_Table(0x01,0x06,0x00,0x78,1);
  delay(2);
  Modbus_Table(0x02,0x06,0x00,0x78,1);
  delay(2);
}


void Joystick_Value(int x, int y, int* lmotor_speed, int* rmotor_speed)
{
  int j_x = x-2050;
  int j_y = y-2055;

  *lmotor_speed = -(j_y + 2.2*((j_x*LENGTH_BET_WHEEL/4)));
  *rmotor_speed = j_y - 2.2*((j_x*LENGTH_BET_WHEEL)/4);  
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
