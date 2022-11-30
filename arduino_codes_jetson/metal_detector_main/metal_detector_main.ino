//////******************METAL_DETECTOR_ROBOT IMR*******////////////

#include <math.h> //this library is for when we are using math functions like sin,cos,tan,squre etc. 
#include <ros.h>   // this is for writing ros publihser and subscriber 
#include <ros/time.h>   // ros library
#include <std_msgs/String.h> // ros library
#include <std_msgs/Empty.h>  // ros library
#include <std_msgs/Int32.h>    // ros library
#include <geometry_msgs/Twist.h>    // ros library for pub. and subs. twist commands
#include <sensor_msgs/ChannelFloat32.h>  
#include <sensor_msgs/JointState.h>  // ros librar for pub. and subs. joint states
#include <Quaternion.h>                // library for conversion angles to quaternion
#include <tf/transform_broadcaster.h>   // for writing tf broadcaster
#include <Servo.h>                   // for servo controlling
#include <Encoder.h>                 // for reading encoder data from pulses
//#include <Wire.h>

#define M1IN1 27        // right motor pin 1
#define M1IN2 29        // right motor pin 2    
#define M1PWM 10        // right motor pwm pin
#define M2IN1 31        // left motor pin 1
#define M2IN2 33        // left motor pin 2
#define M2PWM 9         // left motor pwm pin
#define METALPIN 35     // signal pin for metal detector
 
int arm_pos = 0;     // variable to store the servo position
int pos_min = 0;      //min angle for servo 
int pos_max = 125;    // max angle for servo
bool forward = LOW, backward = LOW, rturn = LOW, lturn = LOW; // variable for control commands 
bool metal_detected = LOW;      // variable for as soft switch 
long current_millis = 0;        // variable for current time 
long pre_millis = 0;            //variable for previous time
long newPositionMR = 0;         // variable for encoder library for 
long newPositionML = 0;
char IMU_DATA[30] = "";
double old_pos_x = 0;
double old_pos_y = 0;
double new_pos_x = 0;
double new_pos_y = 0;
float counts_per_rev = 5600;
float wheel_dia = 0.1524;          // in meter
float wheel_circ = PI * wheel_dia; // wheel cicumference
float wheel_sep = 0.35;            // wheel seperation in meter
float vl = 0.0, vr = 0.0, wl = 0.0, wr = 0.0, r = (wheel_dia / 2.0), theta_l = 0.0, theta_r = 0.0, theta_lo = 0.0, theta_ro = 0.0, w = 0.0, theta = 0.0, thetao = 0.0, v = 0.0;
double current_time = millis();
double last_time = current_time;
double RMR = 0;
double LMR = 0;
long oldPositionMR = -999;
long oldPositionML = -999;
long oldcountL = 0;
long oldcountR = 0;
float xo = 0;
float yo = 0;
float x = 0;
float y = 0;
double s = 0;

ros::Publisher joint_states("joint_states", &robot_state);
ros::NodeHandle nh;

geometry_msgs::TransformStamped t1, t2, t3, t4, t5, t6, t7;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";
char Back_left_wheel_link[] = "/left_wheel_link";
char Back_right_wheel_link[] = "/right_wheel_link";
char Ball_caster_left_link[] = "/Ball_caster_left_link";
char Ball_caster_right_link[] = "/Ball_caster_right_link";
char Lidar_link[] = "/Lidar_link";
char Metal_detector_arm_link[] = "/Metal_detector_arm_link";


// char* robot_id = "metal_detector_bot";
char *robot_id = "";
sensor_msgs::JointState robot_state;
char *a[] = {"left_wheel_joint", "right_wheel_joint", "Ball_caster_left_joint", "Ball_caster_right_joint", "Metal_detector_plate_joint"}; // R: Right - L: Left

float pos[5]; /// stores arduino time
float vel[5];
float eff[5];
Servo myservo; // create servo object to control a servo
// twelve servo objects can be created on most boards


///////////////*****************//////////

void cmd_vel(const geometry_msgs::Twist &twist_msg)
{
  if (twist_msg.linear.x > 0)
  {
    forward = HIGH;
  }
  else if (twist_msg.linear.x < 0)
  {
    backward = HIGH;
  }
  else if (twist_msg.linear.y > 0)
  {
    rturn = HIGH;
  }
  else if (twist_msg.linear.y < 0)
  {
    lturn = HIGH;
  }
  else
  {
    forward = LOW, backward = LOW, rturn = LOW, lturn = LOW;
  }
 

  digitalWrite(13, HIGH - digitalRead(13)); // blink the led
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel);
/////////////////////**********////////////
void RPWM(const std_msgs::Int32 &rpwm_msg)
{

  rpwm_value = rpwm_msg.data;
}

ros::Subscriber<std_msgs::Int32> sub_rpwm("RIGHT_PWM", RPWM);

/////////////////////**************///////////////
void LPWM(const std_msgs::Int32 &lpwm_msg)
{

  rpwm_value = lpwm_msg.data;
}

ros::Subscriber<std_msgs::Int32> sub_lpwm("LEFT_PWM", LPWM);

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
Encoder M1ENC(18, 19);
Encoder M2ENC(2, 3);
//   avoid using pins with LEDs attached

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////Setup Function ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(M1IN1, OUTPUT);
  pinMode(M1IN2, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2IN1, OUTPUT);
  pinMode(M2IN2, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(METALPIN, INPUT);
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
  robot_state.name_length = 5;
  robot_state.velocity_length = 2;
  robot_state.position_length = 5; /// here used for arduino time
  robot_state.effort_length = 2;   /// here used for arduino time
  robot_state.name = a;
  myservo.attach(7); // attaches the servo on pin 9 to the servo object
  delay(20);
  nh.spinOnce();
  myservo.write(0);
  broadcasting_the_tf_trasform(); // Custom made function to publish all the transforms
}


Quaternion Q1, Q2;
void broadcasting_the_tf_trasform()
{
  Quaternion Q1, Q2;
  double a[4];
  Q2 = Q1.from_euler_rotation(-1.5707963267949, -1.17899991460539, -1.5707963267949);

  t2.header.frame_id = base_link;
  t2.child_frame_id = Back_left_wheel_link;
  t2.transform.translation.x = -0.0765;
  t2.transform.translation.y = 0.00190302448688115;
  t2.transform.translation.z = 0.00943214142854351;
  t2.transform.rotation.x = Q2.a;
  t2.transform.rotation.y = Q2.b;
  t2.transform.rotation.z = Q2.c;
  t2.transform.rotation.w = Q2.d;
  t2.header.stamp = nh.now();

  broadcaster.sendTransform(t2);


  Q2 = Q1.from_euler_rotation(1.5707963267949, 1.322, 3.1415927);

  t3.header.frame_id = base_link;
  t3.child_frame_id = Back_right_wheel_link;
  t3.transform.translation.x = -0.1766;
  t3.transform.translation.y = -0.15065;
  t3.transform.translation.z = 0.019;
  t3.transform.rotation.x = Q2.a;
  t3.transform.rotation.y = Q2.b;
  t3.transform.rotation.z = Q2.c;
  t3.transform.rotation.w = Q2.d;
  t3.header.stamp = nh.now();

  broadcaster.sendTransform(t3);


  Q2 = Q1.from_euler_rotation(0.0, 0.0, 0.0);

  t4.header.frame_id = base_link;
  t4.child_frame_id = Lidar_link;
  t4.transform.translation.x = 0.0209;
  t4.transform.translation.y = 0.0;
  t4.transform.translation.z = 0.171;
  t4.transform.rotation.x = Q2.a;
  t4.transform.rotation.y = Q2.b;
  t4.transform.rotation.z = Q2.c;
  t4.transform.rotation.w = Q2.d;
  t4.header.stamp = nh.now();

  broadcaster.sendTransform(t4);


  Q2 = Q1.from_euler_rotation(1.5707963267949, 0.0, 0.0);

  t5.header.frame_id = base_link;
  t5.child_frame_id = Ball_caster_right_link;
  t5.transform.translation.x = -0.06543;
  t5.transform.translation.y = -0.07735;
  t5.transform.translation.z = -0.04811;
  t5.transform.rotation.x = Q2.a;
  t5.transform.rotation.y = Q2.b;
  t5.transform.rotation.z = Q2.c;
  t5.transform.rotation.w = Q2.d;
  t5.header.stamp = nh.now();

  broadcaster.sendTransform(t5);


  Q2 = Q1.from_euler_rotation(1.5707963267949, 0.0, 0.0);

  t6.header.frame_id = base_link;
  t6.child_frame_id = Ball_caster_left_link;
  t6.transform.translation.x = 0.06543;
  t6.transform.translation.y = 0.07735;
  t6.transform.translation.z = -0.04811;
  t6.transform.rotation.x = Q2.a;
  t6.transform.rotation.y = Q2.b;
  t6.transform.rotation.z = Q2.c;
  t6.transform.rotation.w = Q2.d;
  t6.header.stamp = nh.now();

  broadcaster.sendTransform(t6);


  Q2 = Q1.from_euler_rotation(1.5707963267949, 0.0, -3.08892219013656);

  t7.header.frame_id = base_link;
  t7.child_frame_id = Metal_detector_arm_link;
  t7.transform.translation.x = 0.11465;
  t7.transform.translation.y = 0.00025000000000001;
  t7.transform.translation.z = -0.00924999999999999;
  t7.transform.rotation.x = Q2.a;
  t7.transform.rotation.y = Q2.b;
  t7.transform.rotation.z = Q2.c;
  t7.transform.rotation.w = Q2.d;
  t7.header.stamp = nh.now();

  broadcaster.sendTransform(t7);
}


void loop()
{
  nh.spinOnce();

  CAL_VL_VR();

  Q2 = Q1.from_euler_rotation(theta + PI, PI, 0);
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


  RMR = (double(newPositionMR) / double(counts_per_rev)) * (2.0 * 3.146);
  LMR = (double(newPositionML) / double(counts_per_rev)) * (2.0 * 3.146);
  pos[0] = LMR;
  pos[1] = RMR;
  pos[4] = 0;
  vel[0] = 100;
  vel[1] = 100;
  eff[0] = 50;
  eff[1] = 50;

  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;
  robot_state.header.stamp = nh.now();
  joint_states.publish(&robot_state);

  RENC.data = newPositionMR;
  RENCDATA.publish(&RENC);

  LENC.data = newPositionML;
  LENCDATA.publish(&LENC);

  mtl_status.data = digitalRead(METALPIN);
  METAL.publish(&mtl_status);


  if (forward == HIGH)
  {
    digitalWrite(M1IN1, HIGH);
    digitalWrite(M1IN2, LOW);
    analogWrite(M1PWM, 80);
    digitalWrite(M2IN1, HIGH);
    digitalWrite(M2IN2, LOW);
    analogWrite(M2PWM, 80);
    servo_arm();
    ENCNT();
  }
  else if (backward == HIGH)
  {
    digitalWrite(M1IN1, LOW);
    digitalWrite(M1IN2, HIGH);
    analogWrite(M1PWM, 80);
    digitalWrite(M2IN1, LOW);
    digitalWrite(M2IN2, HIGH);
    analogWrite(M2PWM, 80);
    servo_arm();
    ENCNT();
  }
  else if (rturn == HIGH)
  {
    digitalWrite(M1IN1, HIGH);
    digitalWrite(M1IN2, LOW);
    analogWrite(M1PWM, 80);
    digitalWrite(M2IN1, LOW);
    digitalWrite(M2IN2, HIGH);
    analogWrite(M2PWM, 80);
    servo_arm();
    ENCNT();
  }
  else if (lturn == HIGH)
  {

    digitalWrite(M1IN1, LOW);
    digitalWrite(M1IN2, HIGH);
    analogWrite(M1PWM, 80);
    digitalWrite(M2IN1, HIGH);
    digitalWrite(M2IN2, LOW);
    analogWrite(M2PWM, 80);
    servo_arm();
    ENCNT();
  }
  else
  {
    digitalWrite(M1IN1, HIGH);
    digitalWrite(M1IN2, LOW);
    analogWrite(M1PWM, 0);
    digitalWrite(M2IN1, HIGH);
    digitalWrite(M2IN2, LOW);
    analogWrite(M2PWM, 0);
  }

  ENCNT();
  last_time = current_time;
}

void ENCNT()
{

  newPositionMR = M2ENC.read();
  if (newPositionMR != oldPositionMR)
  {
    oldPositionMR = newPositionMR;
  }
  newPositionML = M1ENC.read();
  if (newPositionML != oldPositionML)
  {
    oldPositionML = newPositionML;
  }

}



void CAL_VL_VR()
{
  theta_r = (float(newPositionMR) / counts_per_rev) * (2.0 * 3.146); // angular possition of the wheel
  theta_l = (float(newPositionML) / counts_per_rev) * (2.0 * 3.146);

  current_time = millis();

  wl = ((theta_l - theta_lo) / (current_time - last_time)) * 1000; // angular velocity off the wheel
  wr = ((theta_r - theta_ro) / (current_time - last_time)) * 1000;
  theta_lo = theta_l;
  theta_ro = theta_r;
  vl = r * wl; // linear velocity of the wheel
  vr = r * wr;
  //    Serial.print("LEFT WHEEL VELOCITY: "); Serial.print(vl);
  //    Serial.print("   RIGHT WHEEL VELOCITY: "); Serial.println(vr);

  w = (vr - vl) / wheel_sep; // angular velocity of robot
  v = (vr + vl) / 2.0;       // linear velocity of the robot
  unsigned long Ts = current_time - last_time;
  theta = thetao + (w * Ts) / 1000;
  thetao = theta;
  // Serial.print("   yaw angle of the robot: "); Serial.println(theta);
  x = xo + (v * cos(double(theta)) * Ts) / 1000;
  y = yo + (v * sin(double(theta)) * Ts) / 1000;
  xo = x;
  yo = y;
}

void servo_arm()
{

  if ((current_millis - pre_millis >= 30) && (arm_pos < pos_max)) // goes from 0 degrees to 180 degrees
  {                                                               // in steps of 1 degree
    arm_pos++;
    myservo.write(arm_pos); // tell servo to go to position in variable 'pos'
                            // waits 15ms for the servo to reach the position
    pre_millis = current_millis;
  }
  else if ((current_millis - pre_millis >= 30) && (arm_pos > 0)) // goes from 0 degrees to 180 degrees
  {                                                              // in steps of 1 degree
    arm_pos--;
    myservo.write(arm_pos); // tell servo to go to position in variable 'pos'
                            // waits 15ms for the servo to reach the position
    pre_millis = current_millis;
  }
}
