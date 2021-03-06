#define X   A0
#define Y   A1

#define LENGTH_BET_WHEEL    0.281     // 28.1cm = 0.281m
#define WHEEL_RADIUS        0.0625    // 6.25cm = 0.0625m 

#include <ros.h>
#include <geometry_msgs/Twist.h>

int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
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
  
  if(vx > 0)
  {
    digitalWrite(led_pin_user[0], LOW);
  }
  else if(vx < 0)
  {
    digitalWrite(led_pin_user[0], HIGH);
  }
  
  if(vth > 0)
  {
    digitalWrite(led_pin_user[1], LOW);
  }
  else if(vth < 0)
  {
    digitalWrite(led_pin_user[1], HIGH);
  }

  
}

ros::Subscriber<geometry_msgs::Twist>moblie_vel("/cmd_vel",callback);


void setup() {
  // initialize serial communications at 9600 bps:
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
  }
  Serial.print("sw1 : ");
  Serial.println(sw1_on);
//  Serial.print("sw2 : ");
//  Serial.println(sw2_on);
//  
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

  Serial1.write(protocol_buf, 8);
}

  
