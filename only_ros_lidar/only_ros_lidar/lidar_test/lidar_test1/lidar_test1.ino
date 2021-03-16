#define X   A0
#define Y   A1

#define LENGTH_BET_WHEEL    0.281     // 28.1cm = 0.281m
#define WHEEL_RADIUS        0.0625    // 6.25cm = 0.0625m 

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros_lib/laser_values/caselidar.h>

int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };


char joy_flag = 0;         // 모바일로봇 run flag
int lmotor_speed = 0;
int rmotor_speed = 0;

unsigned char sw1_on;
unsigned char sw2_on = 0;
double vx = 0;
double vth = 0;
int count = 0;

String MODE;
unsigned char mode_num = 0;
double front_min_range = 0;
double left_min_range = 0;
double right_min_range = 0;

ros::NodeHandle nh;

void calb_lidar(const laser_values::caselidar& msg)
{
  MODE = msg.object_pose;
  front_min_range = msg.front_range;   // meter
  left_min_range = msg.left_range;     // meter
  right_min_range = msg.right_range;   // meter

  
}


ros::Subscriber<laser_values::caselidar>lidar_data("lidar_ranges",calb_lidar);


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);

  nh.initNode();
  nh.subscribe(lidar_data);  
}


void loop() {
  
  nh.spinOnce();
  Serial.println(front_min_range);
  Serial.println(left_min_range);
  Serial.println(right_min_range);
  

}
