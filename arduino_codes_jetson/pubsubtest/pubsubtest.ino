/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
ros::NodeHandle  nh;


void messageCb( const std_msgs::Int32& msg){
// String myString = "";
//   myString = msg.data;
// if (myString.substring(11) == "$") {
//  
//  myString = myString.substring(0,11);
//  Serial.println("\t\t\t"+myString);
//}
//else{
//  myString = "";
//  }
if (msg.data==100)
 {Serial.println(100);}

  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Int32> sub("distance", messageCb );



//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//
//char hello[13] = "hello world!";

void setup()
{
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  nh.initNode();
//  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
//  str_msg.data = hello;
//  chatter.publish( &str_msg );
  nh.spinOnce();
//  delay(500);
}
