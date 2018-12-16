
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

Servo servo_steering;
Servo servo_motor;

static int steering_midpoint = 90;
static int motor_midpoint = 90;

void velocity_cb(const geometry_msgs::Twist& cmd_msg){
  servo_motor.write(motor_midpoint + cmd_msg.linear.x * 300);
  servo_steering.write(steering_midpoint + cmd_msg.angular.z * 40);  
}


ros::Subscriber<geometry_msgs::Twist> velocity_sub("/cmd_vel_mux/input/teleop", velocity_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(velocity_sub);
  
  servo_steering.attach(20);
  servo_motor.attach(21);

  servo_steering.write(steering_midpoint);
  servo_motor.write(motor_midpoint);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
