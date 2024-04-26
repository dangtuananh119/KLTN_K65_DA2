#include <ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <Wire.h>
#include <std_msgs/Float32.h>

#define QUAYDUONG_R 4
#define QUAYAM_R 5

#define QUAYDUONG_L 6
#define QUAYAM_L 7

#define ACCEL_XOUTH 0x3B
#define ACCEL_XOUTL 0x3C
#define ACCEL_YOUTH 0x3D
#define ACCEL_YOUTL 0x3E
#define ACCEL_ZOUTH 0x3F
#define ACCEL_ZOUTL 0x40

#define GYRO_XH     0x43
#define GYRO_XL     0X44 

const int MPU = 0x68;

float GyroX, GyroY, GyroZ;
float roll, pitch, yaw, froll, fpitch, fyaw = 0;

float elapsedTime, currentTime, previousTime;
float dt = 0.015;
bool turn_fillter = false;
bool stop = true;

int pulse_r = 0;
int pulse_l = 0;
int pulse_l_before = 0;
int pulse_l_after  = 0;
int pulse_r_before = 0;
int pulse_r_after  = 0;

const int R = 0.08;
float x     = 0.0;
float y     = 0.0;
int the_ta_goal = 0;
float dis_tmp      = 0;
int esp          = 0;
float x_goal = 0.0;
float y_goal = 0.0;
float x_update = 0.0;
float y_update = 0.0;
int flag_move = 0;
int flag_update = 0;

//PID steering
unsigned long time;
const float Kp_the_ta = 2.1667;
const float Ki_the_ta = 0.12;
const float Kd_the_ta = 0.01;
int del_ta     = 0;
int the_ta_set = 0;
int PWM_R_set = 60;
int PWM_L_set = 60;
int sum_error_the_ta = 0;
int et_the_ta        = 0;
int es_the_ta        = 0;

unsigned long time_start = 0;

// ROS 
ros::NodeHandle nh;
geometry_msgs::Point pose_msg;
ros::Publisher pose_pub("pose", &pose_msg);

void count_r()
{
  if (digitalRead(43) == LOW)
  {
     pulse_r ++;
  }
  else
  {
    pulse_r --;
  }
}

void count_l()
{
  if (digitalRead(42) == LOW)
  {    
    pulse_l --;
  }
  else
  {
    pulse_l ++;
  }
}

void run_r(int power_r)
{
  if (power_r >= 0)
  {
    analogWrite(QUAYAM_R, 0);
    analogWrite(QUAYDUONG_R, power_r);
  }
  if (power_r < 0)
  { 
    analogWrite(QUAYDUONG_R, 0);
    analogWrite(QUAYAM_R, power_r);
  }
}

void run_l(int power_l)
{
  if (power_l >= 0)
  {
    analogWrite(QUAYAM_L, 0);
    analogWrite(QUAYDUONG_L, power_l);
  }
  if (power_l < 0)
  { 
    analogWrite(QUAYDUONG_L, 0);
    analogWrite(QUAYAM_L, power_l);
  }
}

void PID_steering(int the_ta)
{
  time = millis();
  the_ta_set = atan2((y_goal - y),(x_goal - x));
  if(time-time_start >= 100)
  {
    del_ta = the_ta_set - the_ta;
    sum_error_the_ta += del_ta;

    es_the_ta  = del_ta;
     
    PWM_R_set  = 60 + Kp_the_ta * del_ta + Ki_the_ta * sum_error_the_ta + Kd_the_ta * (es_the_ta - et_the_ta)/0.1;
    PWM_L_set  = 60 - Kp_the_ta * del_ta - Ki_the_ta * sum_error_the_ta - Kd_the_ta * (es_the_ta - et_the_ta)/0.1;

    if(PWM_R_set < 60)
    {
      PWM_R_set = 60;
    }
    if(PWM_R_set > 255)
    {
      PWM_R_set = 255;
    }
    if(PWM_L_set < 60)
    {
      PWM_L_set = 60;
    }
    if(PWM_L_set > 255)
    {
      PWM_L_set = 255;
    }
    time_start = time ;

    et_the_ta = es_the_ta;
  }
    if(flag_move == 1)
    {
      run_r(PWM_R_set);
      run_l(PWM_L_set);
    }else{
      run_r(0);
      run_l(0);
    }
}

void setup()
{
  nh.initNode();
  nh.advertise(pose_pub);
  ros::Subscriber<geometry_msgs::Point> pose_goal_sub("goal_pose", &pose_goal_Callback);
  ros::Subscriber<geometry_msgs::Point> ekf_sub("ekf_pose", &ekf_pose_Callback);
  nh.subscribe(pose_goal_sub);
  nh.subscribe(ekf_sub);
  attachInterrupt(4, count_r, RISING); 
  attachInterrupt(5, count_l, RISING);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.begin(115200);
}

void pose_goal_Callback(const geometry_msgs::Point& goal_pose_msg)
{
    x_goal = goal_pose_msg.x;
    y_goal = goal_pose_msg.y;
    flag_move = goal_pose_msg.z;
}

void ekf_pose_Callback(const geometry_msgs::Point& ekf_pose_msg)
{
    x_update = ekf_pose_msg.x;
    y_update = ekf_pose_msg.y;
    flag_update = ekf_pose_msg.z;
}

void loop()
{
  //Get_angle
  if(turn_fillter == false)
  {
    filter_gyro();
    turn_fillter = true;
  }

  previousTime = millis();
  Wire.beginTransmission(MPU);
  Wire.write(GYRO_XH);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) /131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) /131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) /131.0;
  roll += (GyroX - froll) *dt;
  pitch += (GyroY - fpitch) *dt;
  yaw  += (GyroZ - fyaw)  *dt;

  if(yaw > 360)
  {
    yaw -= 360;
  }
  else if(yaw < -360)
  {
    yaw += 360;
  }

  double the_ta = -yaw;

 //_________________________ODOMETRY_______________________________________
    pulse_l_after = pulse_l;
    pulse_r_after = pulse_r;
    float d_center = (((pulse_l_after - pulse_l_before) + (pulse_r_after - pulse_r_before))* 0.036)/200; // (m)
    x              += d_center * cos(the_ta * 0.0174533); // (m)
    y              += d_center * sin(the_ta * 0.0174533); // (m)
    
    if(flag_update == 1)
    {  
      x = x_update;
      y = y_update;
    }

    PID_steering(the_ta);

    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.z = the_ta*0.0174533;
    pose_pub.publish(&pose_msg);

    pulse_r_before = pulse_r_after;
    pulse_l_before = pulse_l_after;

    currentTime = millis();
    elapsedTime = currentTime - previousTime;
    dt = elapsedTime/ 1000;

    nh.spinOnce();

} 

void filter_gyro()
{
  Serial.println("Start probe filter");
  delay(500);
  for(int x = 0; x < 10; x++)
  {
    Serial.print("*");
    delay(100);
  }
  Serial.println("*"); 
  for(int x = 0; x < 1000; x++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read())/131.0;
    GyroY = (Wire.read() << 8 | Wire.read())/131.0;
    GyroZ = (Wire.read() << 8 | Wire.read())/131.0;
    froll += GyroX;
    fpitch += GyroY;
    fyaw  += GyroZ;
  }
  froll  = froll / 1000;
  fpitch = fpitch / 1000;
  fyaw   = fyaw  / 1000;
  Serial.println("Probe filter done!");
  Serial.print("X filter : "); Serial.println(froll);
  Serial.print("Y filter : "); Serial.println(fpitch);
  Serial.print("Z filter : "); Serial.println(fyaw);
} 