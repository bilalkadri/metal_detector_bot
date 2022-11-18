/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */


 
#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/JointState.h>
#include <Quaternion.h>
#include <tf/transform_broadcaster.h>



ros::NodeHandle  nh;

geometry_msgs::TransformStamped t1,t2,t3,t4,t5;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";
char Left_wheel_link[] = "/Left_wheel_link";
char Right_wheel_link[] = "/Right_wheel_link";
char Ball_caster_front_link[] = "/Ball_caster_front_link";
char Ball_caster_back_link[] = "/Ball_caster_back_link";


#define M1IN1 27
#define M1IN2 29
#define M1PWM 10
#define M2IN1 31
#define M2IN2 33
#define M2PWM 9
#define METALPIN 35
#include <Encoder.h>

//char* robot_id = "metal_detector_bot";
char* robot_id = "";
sensor_msgs::JointState robot_state;
char *a[] = {"Left_wheel_joint", "Right_wheel_joint","Ball_caster_front_joint","Ball_caster_back_joint"};  //R: Right - L: Left
float pos[4]; /// stores arduino time
float vel[4];
float eff[4];

ros::Publisher joint_states("joint_states", &robot_state);
/*
MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011-2016 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/


// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,-1,-1,-1,1,1,1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)

#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

int rpwm_value = 0;
int lpwm_value = 0;
bool forward = LOW,backward = LOW,rturn = LOW,lturn= LOW;
bool metal_detected = LOW;

///////////////*****************//////////
void cmd_vel( const geometry_msgs::Twist& twist_msg){
if(twist_msg.linear.x>0)
{forward=HIGH; }
 else if( twist_msg.linear.x<0)
 { backward=HIGH; }
  else if( twist_msg.linear.y>0)
 { rturn=HIGH; }
 else if( twist_msg.linear.y<0)
 { lturn=HIGH; }
 else
 { 
  forward = LOW,backward = LOW,rturn = LOW,lturn= LOW;
  }
//if (frd_msg.data == 1)
//{
//  frwrd = HIGH;
//  }
//  else {
//    frwrd = LOW;}

  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel);
/////////////////////**********////////////
void RPWM( const std_msgs::Int32& rpwm_msg){

rpwm_value = rpwm_msg.data ;
}

ros::Subscriber<std_msgs::Int32> sub_rpwm("RIGHT_PWM", RPWM );

/////////////////////**************///////////////
void LPWM( const std_msgs::Int32& lpwm_msg){

rpwm_value = lpwm_msg.data ;
}

ros::Subscriber<std_msgs::Int32> sub_lpwm("LEFT_PWM", LPWM );

std_msgs::Int32 RENC;
ros::Publisher RENCDATA("RENCDATA", &RENC);

std_msgs::Int32 LENC;
ros::Publisher LENCDATA("LENCDATA", &LENC);

std_msgs::Int32 mtl_status;
ros::Publisher METAL("METAL", &mtl_status);

std_msgs::Int32 arm_agnle;
ros::Publisher ARM("ARM", &arm_agnle);

std_msgs::String imu_data;
ros::Publisher IMU("IMU", &imu_data);



// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder  M1ENC(18,19);
Encoder  M2ENC(2,3);
//   avoid using pins with LEDs attached
long newPositionMR = 0;
long newPositionML = 0;
char IMU_DATA[30] = "";
double old_pos_x = 0;
double old_pos_y = 0;
double new_pos_x = 0;
double new_pos_y = 0;
float counts_per_rev = 5500;
float wheel_dia = 0.1524 ;                 //in meter
float wheel_circ = PI*wheel_dia; //wheel cicumference
float wheel_sep = 0.2794   ;             //wheel seperation in meter

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////Setup Function ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


void setup() {
pinMode(M1IN1,OUTPUT);
pinMode(M1IN2,OUTPUT);
pinMode(M1PWM,OUTPUT);
pinMode(M2IN1,OUTPUT);
pinMode(M2IN2,OUTPUT);
pinMode(M2PWM,OUTPUT);
pinMode(METALPIN,INPUT);
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  nh.initNode();
   nh.advertise(IMU);
  nh.advertise(RENCDATA);
  nh.advertise(LENCDATA);
  nh.advertise(METAL);
  nh.advertise(ARM);
  nh.subscribe(sub);
    nh.subscribe(sub_rpwm);
  nh.subscribe(sub_lpwm);
  nh.advertise(joint_states);
    broadcaster.init(nh);
    robot_state.header.frame_id = robot_id;
  robot_state.name_length = 4;
  robot_state.velocity_length = 2;
  robot_state.position_length = 4; /// here used for arduino time
  robot_state.effort_length = 2; /// here used for arduino time

    robot_state.name = a;
  
///  pinMode (STATUS_LED,OUTPUT);  // Status LED

  I2C_Init();

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

///  digitalWrite(STATUS_LED,LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
    {
       
    Read_Gyro();
    Read_Accel();
     Serial.println("IMU INITIALIZED");
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  counter=0;
nh.spinOnce();

//  now();
//  robot_state.header.stamp = nh.now();



 broadcasting_the_tf_trasform(); //Custom made function to publish all the transforms
}
  double current_time = millis();
  double last_time = current_time;

double RMR = 0;
double LMR = 0;
long oldPositionMR  = -999;
long oldPositionML  = -999;
long oldcountL = 0;
long oldcountR = 0;
float xo = 0;
float yo = 0;
float x = 0;
float y = 0;
double s = 0; 

 Quaternion Q1,Q2;
void  broadcasting_the_tf_trasform()
  {
  /*Broadcasting the transform between odom and base_link*/
   
//  t1.header.frame_id = odom;
//  t1.child_frame_id = base_link;
//  t1.transform.translation.x = 1.0; 
//  t1.transform.rotation.x = 0.0;
//  t1.transform.rotation.y = 0.0; 
//  t1.transform.rotation.z = 0.0; 
//  t1.transform.rotation.w = 1.0;  
//  t1.header.stamp = nh.now();
// 
//  broadcaster.sendTransform(t1);

  /*Broadcasting the transform between base_link and Left_wheel_link*/
  
  Quaternion Q1,Q2;
  double a[4];
  Q2=Q1.from_euler_rotation(-1.5707963267949, -1.17899991460539, 1.5707963267949);
  
  t2.header.frame_id = base_link;
  t2.child_frame_id = Left_wheel_link;
  t2.transform.translation.x = -0.0765;
  t2.transform.translation.y = 0.00190302448688115;
  t2.transform.translation.z = 0.00943214142854351;
  t2.transform.rotation.x = Q2.a;
  t2.transform.rotation.y = Q2.b; 
  t2.transform.rotation.z = Q2.c; 
  t2.transform.rotation.w = Q2.d;  
  t2.header.stamp = nh.now();
  
  broadcaster.sendTransform(t2);

  Q2=Q1.from_euler_rotation(1.5707963267949, -0.566329337388945, -1.5707963267949);
  
  t3.header.frame_id = base_link;
  t3.child_frame_id = Right_wheel_link;
  t3.transform.translation.x = 0.0764999999999999;
  t3.transform.translation.y = 0.00190302448688115;
  t3.transform.translation.z = 0.00943214142854353;
  t3.transform.rotation.x = Q2.a;
  t3.transform.rotation.y = Q2.b; 
  t3.transform.rotation.z = Q2.c; 
  t3.transform.rotation.w = Q2.d;  
  t3.header.stamp = nh.now();
  
  broadcaster.sendTransform(t3);


   Q2=Q1.from_euler_rotation(1.5707963267949, 0.0, 0.0);
  
  t4.header.frame_id = base_link;
  t4.child_frame_id = Ball_caster_front_link;
  t4.transform.translation.x = -0.0001379719182397;
  t4.transform.translation.y = 0.0700435036565304;
  t4.transform.translation.z = -0.014;
  t4.transform.rotation.x = Q2.a;
  t4.transform.rotation.y = Q2.b; 
  t4.transform.rotation.z = Q2.c; 
  t4.transform.rotation.w = Q2.d;  
  t4.header.stamp = nh.now();
  
  broadcaster.sendTransform(t4);

     Q2=Q1.from_euler_rotation(1.5707963267949, 0.0, 0.0);
  
  t5.header.frame_id = base_link;
  t5.child_frame_id = Ball_caster_back_link;
  t5.transform.translation.x = -0.0001379719182397;
  t5.transform.translation.y = -0.0700435036565304;
  t5.transform.translation.z = -0.014;
  t5.transform.rotation.x = Q2.a;
  t5.transform.rotation.y = Q2.b; 
  t5.transform.rotation.z = Q2.c; 
  t5.transform.rotation.w = Q2.d;  
  t5.header.stamp = nh.now();
  
  broadcaster.sendTransform(t5);
    /*Broadcasting the transform between base_link and Right_wheel_link*/
    /*Broadcasting the transform between base_link and Ball_caster_back_link*/
    /*Broadcasting the transform between base_link and Ball_caster_forward_link*/
   //Have to write the code here
  }




//void publish_odom_frame()
//  {
//
//  double current_time;
//  current_time=nh.now();
//
//
//    
//  }

float vl=0.0,vr=0.0,wl=0.0,wr=0.0,r=0.075/2,theta_l=0.0,theta_r=0.0,theta_lo=0.0,theta_ro=0.0,w=0.0,theta=0,thetao=0.0,v=0.0;

void loop() { 
nh.spinOnce();

 CAL_VL_VR();
// pos_cal();
//  x = float(s)*(float(cos(theta))) + xo; 
//  y = float(s)*(float(cos(theta))) + yo; 
  Q2=Q1.from_euler_rotation(theta+1.5707,PI,0);
  t1.header.frame_id = odom;
  t1.child_frame_id = base_link;
  t1.transform.translation.y = double(y); 
  t1.transform.translation.x = double(x); 
  t1.transform.rotation.x = Q2.a;
  t1.transform.rotation.y = Q2.b; 
  t1.transform.rotation.z = Q2.c; 
  t1.transform.rotation.w = Q2.d;  
  t1.header.stamp = nh.now();
 
  broadcaster.sendTransform(t1);

  
 
       
//    robot_state.velocity = vel;
//    robot_state.effort = eff;
    
   RMR = (double(newPositionMR)/1770.0)*(2.0*3.146);
   LMR = (double(newPositionML)/1770.0)*(2.0*3.146);
    pos[0] = RMR;
    pos[1] =LMR;
    vel[0]= 100;
    vel[1]= 100;
    eff[0]= 50;
    eff[1]= 50;
    
    robot_state.position = pos;
    robot_state.velocity = vel;
    robot_state.effort = eff;
    robot_state.header.stamp = nh.now();
    joint_states.publish( &robot_state );
    
  RENC.data = newPositionMR;
  RENCDATA.publish( &RENC );
  
  LENC.data = newPositionML;
  LENCDATA.publish( &LENC );

  mtl_status.data = digitalRead(METALPIN);
  METAL.publish( &mtl_status );
//String  imudata =  String(ToDeg(roll)) + "," + String(ToDeg(pitch)) + "," + String(ToDeg(yaw));
//  imudata.toCharArray(IMU_DATA, 30);
//  imu_data.data =IMU_DATA ;
//  IMU.publish( &imu_data );
//    arm_angle.data = pos;
//  ARM.publish( &arm_angle );
  
 if (forward==HIGH) 
  {
  digitalWrite(M1IN1,HIGH);
  digitalWrite(M1IN2,LOW);
  analogWrite(M1PWM,80);
  digitalWrite(M2IN1,HIGH);
  digitalWrite(M2IN2,LOW);
  analogWrite(M2PWM,80);
   ENCNT(); 
  }
  else if (backward==HIGH) 
  {
  digitalWrite(M1IN1,LOW);
  digitalWrite(M1IN2,HIGH);
  analogWrite(M1PWM,80);
  digitalWrite(M2IN1,LOW);
  digitalWrite(M2IN2,HIGH);
  analogWrite(M2PWM,80);
   ENCNT(); 
  }
  else if (rturn==HIGH) 
  {
  digitalWrite(M1IN1,HIGH);
  digitalWrite(M1IN2,LOW);
  analogWrite(M1PWM,80);
  digitalWrite(M2IN1,LOW);
  digitalWrite(M2IN2,HIGH);
  analogWrite(M2PWM,80);
   ENCNT(); 
  }
  else if (lturn==HIGH) 
  {

    digitalWrite(M1IN1,LOW);
  digitalWrite(M1IN2,HIGH);
  analogWrite(M1PWM,80);
  digitalWrite(M2IN1,HIGH);
  digitalWrite(M2IN2,LOW);
  analogWrite(M2PWM,80);
   ENCNT(); 
  }
 else { digitalWrite(M1IN1,HIGH);
  digitalWrite(M1IN2,LOW);
  analogWrite(M1PWM,0);
  digitalWrite(M2IN1,HIGH);
  digitalWrite(M2IN2,LOW);
  analogWrite(M2PWM,0);

  }
//  delay(100);/
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
    {
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;



    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***

//    printdata();
  }
  ENCNT(); 
 last_time = current_time;
}
void ENCNT()
{

   newPositionMR = M2ENC.read();
  if (newPositionMR != oldPositionMR) {
    oldPositionMR = newPositionMR;
  }
       newPositionML = M1ENC.read();
  if (newPositionML != oldPositionML) {
    oldPositionML = newPositionML;
  }
//    Serial.print(newPositionMR);Serial.print("\t");
//    Serial.println(newPositionML);
  
//  if ((newPositionMR-oldcountR)>=1770)
//  {  
//    RMR++;
//    oldcountR = newPositionMR;
//  }
//  else if((newPositionMR-oldcountR)<=-1770)
//    {
//      RMR--;
//    oldcountR = newPositionMR;
//    }
// 
//  if ((newPositionML-oldcountL)>=1770)
//  { LMR++;
//  oldcountL = newPositionML;
//  }
//  else if((newPositionML-oldcountL)<=-1770)
//    {
//      LMR--;
//     oldcountL = newPositionML;
//    }

//    Serial.print(RMR);Serial.print("\t");
//    Serial.println(LMR);
}

void pos_cal()
{
// double left_wheel_dist = (double(newPositionML)/double(counts_per_rev))*double(wheel_circ);
//  double right_wheel_dist = (double(newPositionMR)/double(counts_per_rev))*double(wheel_circ);
//    s = (left_wheel_dist+right_wheel_dist)/2;
//    theta = ((right_wheel_dist-left_wheel_dist)/(double(wheel_sep)))+double(theta);

//  theta = (theta*PI)/180.0;
  
  }



void CAL_VL_VR()
{
 theta_l  = (float(newPositionMR)/1770.0)*(2.0*3.146); // angular possition of the wheel
 theta_r = (float(newPositionML)/1770.0)*(2.0*3.146);
    
current_time = millis();

wl=((theta_l-theta_lo)/(current_time-last_time))*1000; // angular velocity off the wheel 
wr=((theta_r-theta_ro)/(current_time-last_time))*1000;
theta_lo=theta_l;
theta_ro =theta_r;
vl = r*wl;                        //linear velocity of the wheel
vr = r*wr;
//    Serial.print("LEFT WHEEL VELOCITY: "); Serial.print(vl); 
//    Serial.print("   RIGHT WHEEL VELOCITY: "); Serial.println(vr); 

w = (vr-vl)/wheel_sep;   // angular velocity of robot
v = (vr+vl)/2;           // linear velocity of the robot
 unsigned long Ts = current_time-last_time;
theta = thetao + (w*Ts)/1000;
thetao = theta;
//Serial.print("   yaw angle of the robot: "); Serial.println(theta); 
x = xo + (v*cos(theta)*Ts)/1000;
y = yo + (v*sin(theta)*Ts)/1000;
   xo = x;
   yo = y;
} 
