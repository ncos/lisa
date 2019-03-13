
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
static int motor_pwm_pin = 21;
static int motor_dir_pin = 22;

Servo servo_steering;

static int steering_midpoint = 90;
static unsigned int watchdog = 0;

void velocity_cb(const geometry_msgs::Twist& cmd_msg){
  watchdog = 0;
  servo_steering.write(steering_midpoint + cmd_msg.angular.z * 40);
  if (cmd_msg.linear.x > 0) {
    digitalWrite(motor_dir_pin, HIGH);
  } else {
    digitalWrite(motor_dir_pin, LOW);
  }
  analogWrite(motor_pwm_pin, int(abs(cmd_msg.linear.x)));
}

void reset_car() {
  digitalWrite(motor_pwm_pin, LOW);  
  digitalWrite(motor_dir_pin, HIGH);
  analogWrite(motor_pwm_pin, 0);
  servo_steering.write(steering_midpoint);
}


ros::Subscriber<geometry_msgs::Twist> velocity_sub("/cmd_vel_mux/motor_raw", velocity_cb);

void setup(){
  pinMode(13, OUTPUT); // led
  pinMode(motor_pwm_pin, OUTPUT); // motor pwm
  pinMode(motor_dir_pin, OUTPUT); // motor direction
  digitalWrite(motor_pwm_pin, LOW);  
  digitalWrite(motor_dir_pin, HIGH);

  nh.initNode();
  nh.subscribe(velocity_sub);
  
  servo_steering.attach(20);
  servo_steering.write(steering_midpoint);

  watchdog = 0;
}

void loop(){
  nh.spinOnce();
  delay(1);
  watchdog ++;

  if (watchdog >= 1000) {
    reset_car();
  }
}
