//new coordinates to the robot ::
//x-axis (Forward of robot)
//             ^
//             |
//             |
//             |
//(y-axis)<--------
//header files for ros
#include <ros.h>
#include <cmath>
//include pos_msg
#include <geometry_msgs/Pose.h>

//include imu msg
#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/Vector3.h>

//timer
#include<Timer.h>
///////////////////////////////////////////imu angles/////////////////////////////
 
 float yaw =0;
 float pitch=0;
 float roll=0;
 float gyro_w=0;
 
////////////////////////////////////////////////func to be a parameter to subscriber//////////////////////////////////////////////////
void handle_imu(const sensor_msgs::Imu &angle)
{ 
       roll = angle.orientation.x;
       pitch = angle.orientation.y;
       yaw = angle.orientation.z; //yaw angle (radian)
       gyro_w = angle.orientation.w; 
}


//initalize node 
ros::NodeHandle nh;

//declare a variable of type pose
geometry_msgs::Pose pose_msgs;

//publish the msg of type pos which include (pos in x,y,orientation)
ros::Publisher pose("pose",&pose_msgs);

//subscriber 
ros::Subscriber<sensor_msgs::Imu> imu ("imu", &handle_imu);

/*struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
}*euler;*/

//////////////////////////////////////////////////////////////Quaternion to euler////////////////////////////////////////////////////////////////
float ToEulerAngles(sensor_msgs::Imu angle) {
    angle.orientation.z=yaw;
    angle.orientation.x=roll;
    angle.orientation.y=pitch;
    angle.orientation.w=gyro_w;
   /* // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler->roll = std::atan2(sinr_cosp, cosr_cosp);
      // pitch (y-axis rotation)
     double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        euler->pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler->pitch = std::asin(sinp);*/

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (angle.orientation.w * angle.orientation.z + angle.orientation.x * angle.orientation.y);
    double cosy_cosp = 1 - 2 * (angle.orientation.y * angle.orientation.y + angle.orientation.z * angle.orientation.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

 return yaw;
}
/***** Prepairing Encoder *****/
// Define encoder X pins
#define PIN_ENCODER_X_A PB_12
#define PIN_ENCODER_X_B PB_13
// Define encoder Y pins
#define PIN_ENCODER_Y_A PB_14
#define PIN_ENCODER_Y_B PB_15

// Define Encoders settings
#define ENCODER_RESOLUTION 2400.0 
#define WHEEL_DIAMETER 10.0  // in cm
// timer settings
#define PERIOD 20

// Encoders X & Y vars

long long counter_x = 0;
long long counter_y = 0;

double current_distance_x;
double current_distance_y = 0;

double x_corrected = 0;
double y_corrected = 0;

double last_distance_x = 0;
double last_distance_y = 0;

double delta_distance_x = 0;
double delta_distance_y = 0;
/*from imu
double theta_radian = 0 ;
double yaw_degree = 0 ;*/

// Define TTL pins
#define PIN_TTL_RX PB_11
#define PIN_TTL_TX PB_10


HardwareSerial Serial3(PIN_TTL_RX, PIN_TTL_TX);
HardwareSerial Serial1(PA_10, PA_9);


Timer calculation_timer;

void setup() {
  Serial1.begin(9600);
  
  nh.initNode();        //intialize node
  nh.advertise(pose);   //advertise topic

  // Encoder X
  pinMode(PIN_ENCODER_X_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_X_B, INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_X_A, encoderPinChangeAx, CHANGE);
  attachInterrupt(PIN_ENCODER_X_B, encoderPinChangeBx, CHANGE);
  // Encoder Y
  pinMode(PIN_ENCODER_Y_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_Y_B, INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_Y_A, encoderPinChangeAy, CHANGE);
  attachInterrupt(PIN_ENCODER_Y_B, encoderPinChangeBy, CHANGE);

  calculation_timer.every(PERIOD, calcuate_Distance);

  // Controlling built in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

}

void loop() {
  calculation_timer.update();
  delay(10);
  
 ////////////////////////////////////////////////////////////////////publish pos msg///////////////////////////////////////////////////////////////////////////
  pose_msgs.position.x = x_corrected;
  pose_msgs.position.y = y_corrected;
  pose_msgs.orientation.z = yaw;
  pose.publish(&pose_msgs);     //publish pos msg
  nh.spinOnce();                //handle ross communication
 ////////////////////////////////////////////////////////////////////subscribe imu msg////////////////////////////////////////////////////
  nh.subscribe(imu);
}

// Pulse-Counting ISRs
//ISRs for x
void encoderPinChangeAx(void)
{
  counter_x += digitalRead(PIN_ENCODER_X_A) == digitalRead(PIN_ENCODER_X_B) ? -1 : 1;
}
void encoderPinChangeBx(void)
{
  counter_x += digitalRead(PIN_ENCODER_X_A) != digitalRead(PIN_ENCODER_X_B) ? -1 : 1;
}

//ISRs for y
void encoderPinChangeAy(void)
{
  counter_y += digitalRead(PIN_ENCODER_Y_A) == digitalRead(PIN_ENCODER_Y_B) ? -1 : 1;
}
void encoderPinChangeBy(void)
{
  counter_y += digitalRead(PIN_ENCODER_Y_A) != digitalRead(PIN_ENCODER_Y_B) ? -1 : 1;
}

//function to calculate distances
void calcuate_Distance()
{
  current_distance_x = -1.0 * counter_x * M_PI * WHEEL_DIAMETER / ENCODER_RESOLUTION;   // 7tet negative 34an el-encoder pins m3kosa.
  current_distance_y = 1.0 * counter_y * M_PI * WHEEL_DIAMETER / ENCODER_RESOLUTION;

  delta_distance_x = current_distance_x  - last_distance_x ;
  delta_distance_y = current_distance_y  - last_distance_y ;

  x_corrected += ( delta_distance_x * cos(yaw) ) - ( delta_distance_y * sin(yaw) );
  y_corrected += ( delta_distance_x * sin(yaw) ) + ( delta_distance_y * cos(yaw) ) ;

  last_distance_x = current_distance_x ;
  last_distance_y = current_distance_y ;

}
