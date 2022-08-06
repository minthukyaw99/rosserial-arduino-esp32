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

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo leftServo;
Servo rightServo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  leftServo.write(cmd_msg.data); //set servo angle, should be from 0-180 
  rightServo.write(cmd_msg.data);
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup(){

  nh.initNode();
  nh.subscribe(sub);
  
  leftServo.attach(9); //attach it to pin 9
  leftServo.write(0);

  rightServo.attach(10);
  rightServo.write(0);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
