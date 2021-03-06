#define X   A0
#define Y   A1

#define LENGTH_BET_WHEEL    0.281     // 28.1cm = 0.281m
#define WHEEL_RADIUS        0.0625    // 6.25cm = 0.0625m 

#include <ros.h>
#include <geometry_msgs/Twist.h>
char joy_flag = 0;         // 모바일로봇 run flag
int lmotor_speed = 0;
int rmotor_speed = 0;

ros::NodeHandle nh;


void callback(const geometry_msgs::Twist& msg)
{
  Serial.println(msg.linear.x);
  Serial.println(msg.linear.y);
  Serial.println(msg.linear.z);
  
}

ros::Subscriber<geometry_msgs::Twist>sub("/cmd_vel",callback);




void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  analogReadResolution(12);

  nh.initNode();
  nh.subscribe(sub);  
}


void loop() {
  int x,y;
  x=analogRead(X);
  y=analogRead(Y);


  nh.spinOnce();

  
  Joystick_Value(x,y,&lmotor_speed,&rmotor_speed);
  Joystick_Control(&lmotor_speed,&rmotor_speed);
  
  
//  Serial.print(" X = ");
//  Serial.print(lmotor_speed);
//
//  Serial.print(", Y = ");
//  Serial.println(rmotor_speed);

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
    delay(20);
    Modbus_Table(0x02,0x06,0x00,0x79,*rmotor_speed);
    delay(20);

    if(joy_flag)
    {
      Modbus_Table(0x01,0x06,0x00,0x78,1);
      delay(20);
      Modbus_Table(0x02,0x06,0x00,0x78,1);
      delay(20);

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

  //Serial1.write(protocol_buf, 8);
}

  
