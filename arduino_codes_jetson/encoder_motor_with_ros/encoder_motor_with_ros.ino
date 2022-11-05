/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

 #include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
ros::NodeHandle  nh;

#define M1IN1 27
#define M1IN2 29
#define M1PWM 10
#define M2IN1 31
#define M2IN2 33
#define M2PWM 9
#include <Encoder.h>

bool frwrd = LOW;

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
if (msg.data == 1)
{
  frwrd = HIGH;
  Serial.println(frwrd);
  }
  else {
    frwrd = LOW;}

  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Int32> sub("forward", messageCb );

std_msgs::Int32 int_msg;
ros::Publisher encdata("encdata", &int_msg);



// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder  M1ENC(18,19);
Encoder  M2ENC(2,3);
//   avoid using pins with LEDs attached
long newPositionM1 = 0;
long newPositionM2 = 0;

void setup() {
pinMode(M1IN1,OUTPUT);
pinMode(M1IN2,OUTPUT);
pinMode(M1PWM,OUTPUT);
pinMode(M2IN1,OUTPUT);
pinMode(M2IN2,OUTPUT);
pinMode(M2PWM,OUTPUT);
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(encdata);
  nh.subscribe(sub);
}

long oldPositionM1  = -999;
long oldPositionM2  = -999;

void loop() { 
nh.spinOnce();
  int_msg.data = newPositionM1;
  encdata.publish( &int_msg );
 if (frwrd==HIGH) 
  {
  digitalWrite(M1IN1,HIGH);
  digitalWrite(M1IN2,LOW);
  analogWrite(M1PWM,255);
  digitalWrite(M2IN1,HIGH);
  digitalWrite(M2IN2,LOW);
  analogWrite(M2PWM,255);
   ENCNT(); 
  }
 else { digitalWrite(M1IN1,HIGH);
  digitalWrite(M1IN2,LOW);
  analogWrite(M1PWM,0);
  digitalWrite(M2IN1,HIGH);
  digitalWrite(M2IN2,LOW);
  analogWrite(M2PWM,0);
  ENCNT(); 
  }
  delay(100);
}
void ENCNT()
{

   newPositionM1 = M1ENC.read();
  if (newPositionM1 != oldPositionM1) {
    oldPositionM1 = newPositionM1;
  }
       newPositionM2 = M2ENC.read();
  if (newPositionM2 != oldPositionM2) {
    oldPositionM2 = newPositionM2;
  }
    Serial.print(newPositionM1);Serial.print("\t");
    Serial.println(newPositionM2);
  

}
