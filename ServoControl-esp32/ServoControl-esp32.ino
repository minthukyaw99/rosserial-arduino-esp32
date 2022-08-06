/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ESP32Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo leftServo;
Servo rightServo;

int leftServoPin = 18;
int rightServoPin = 19;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  leftServo.write(cmd_msg.data); //set servo angle, should be from 0-180 
  rightServo.write(cmd_msg.data);
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  leftServo.setPeriodHertz(50);    // standard 50 hz servo
  rightServo.setPeriodHertz(50);
  leftServo.attach(leftServoPin, 500, 2450);
  rightServo.attach(rightServoPin, 500, 2450);
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}
