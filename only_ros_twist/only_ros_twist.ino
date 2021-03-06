#define LENGTH_BET_WHEEL    0.281     // 28.1cm = 0.281m
#define WHEEL_RADIUS        0.0625    // 6.25cm = 0.0625m 

#include <ros.h>
#include <geometry_msgs/Twist.h>

int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
double vx = 0;
double vth = 0;

ros::NodeHandle nh;


void callback(const geometry_msgs::Twist& msg)
{
  vx = msg.linear.x;
  vth = msg.angular.z;
    
  if(vx > 0)
  {
    digitalWrite(led_pin_user[0], HIGH);
  }
  else if(vx < 0)
  {
    digitalWrite(led_pin_user[0], LOW);
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

ros::Subscriber<geometry_msgs::Twist>sub("/cmd_vel",callback);




void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);

  nh.initNode();
  nh.subscribe(sub);  
}


void loop() {

  nh.spinOnce();

}
  
