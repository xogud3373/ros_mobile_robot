#define X   A0
#define Y   A1

#define LENGTH_BET_WHEEL    0.281     // 28.1cm = 0.281m
#define WHEEL_RADIUS        0.0625    // 6.25cm = 0.0625m 

#include <ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ros_lib/laser_values/caselidar.h>

int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };


char joy_flag = 0;         // 모바일로봇 run flag
int lmotor_speed = 0;
int rmotor_speed = 0;

String MODE;
unsigned char mode_num = 0;
float front_min_range = 0;
float left_min_range = 0;
float right_min_range = 0;


ros::NodeHandle nh;

void Take_action(unsigned char mode_num)
{
  switch(mode_num)
  {
    case 0:
      digitalWrite(led_pin_user[0], HIGH);
      delay(10);
      digitalWrite(led_pin_user[0], LOW);
      delay(10);
      break;
    case 1:
      digitalWrite(led_pin_user[1], HIGH);
      delay(10);
      digitalWrite(led_pin_user[1], LOW);
      delay(10);
      
      break;
    case 2:
      digitalWrite(led_pin_user[2], HIGH);
      delay(10);
      digitalWrite(led_pin_user[2], LOW);
      delay(10);
      
      break;
    case 3:
      digitalWrite(led_pin_user[3], HIGH);
      delay(10);
      digitalWrite(led_pin_user[3], LOW);
      delay(10);
      
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
  //Serial.begin(115200);


  pinMode(led_pin_user[0], OUTPUT);
  pinMode(led_pin_user[1], OUTPUT);
  pinMode(led_pin_user[2], OUTPUT);
  pinMode(led_pin_user[3], OUTPUT);

  nh.initNode();
  nh.subscribe(lidar_data);  
}


void loop() {
  nh.spinOnce();


}
