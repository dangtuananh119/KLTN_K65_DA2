#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
std_msgs::Float32 roll_msg;   //x
std_msgs::Float32 pitch_msg;  //y
std_msgs::Float32 yaw_msg;    //z
std_msgs::Int32   count_wr_msg;
std_msgs::Int32   count_wl_msg;

//ros::Publisher roll_pub("roll_data", &roll_msg);
//ros::Publisher pitch_pub("pitch_data", &pitch_msg);
ros::Publisher yaw_pub("yaw_data", &yaw_msg);
ros::Publisher pubr("pulse_r_count", &count_wr_msg);
ros::Publisher publ("pulse_l_count", &count_wl_msg);

#define ACCEL_YOUTL 0x3E
#define ACCEL_ZOUTH 0x3F
#define ACCEL_ZOUTL 0x40

#define GYRO_XH     0x43
#define GYRO_XL     0X44 

#define QUAYDUONG_R   10
#define QUAYAM_R      9

#define QUAYDUONG_L  11
#define QUAYAM_L     12

const int MPU = 0x68;

float GyroX, GyroY, GyroZ;
float roll, pitch, yaw, froll, fpitch, fyaw = 0;

float elapsedTime, currentTime, previousTime;
float dt = 0.015;
bool turn_fillter = false;

int pulse_r = 0;
int pulse_l = 0;

void count_r()
{
  if(digitalRead(3) == 1)
  {
    pulse_r++;
  }
  else{
    pulse_r--;
  }
}

void count_l()
{
  if(digitalRead(5) == 1)
  {
    pulse_l++;
  }
  else{
    pulse_l--;
  }
}

void run_r(int power_r)
{
  if(power_r >=0)
  {
    analogWrite(QUAYAM_R,0);
    analogWrite(QUAYDUONG_R, power_r);
  }
  if(power_r <0)
  {
    analogWrite(QUAYDUONG_R, 0);
    analogWrite(QUAYAM_R, power_r);
  }
}

void run_l(int power_l)
{
  if(power_l >=0)
  {
    analogWrite(QUAYAM_L, 0);
    analogWrite(QUAYDUONG_L, power_l);
  }
  if(power_l < 0)
  {
    analogWrite(QUAYDUONG_L,0);
    analogWrite(QUAYAM_L, power_l);
  }
}
 
void setup()
{
  attachInterrupt(0, count_r, RISING);
  attachInterrupt(1, count_l, RISING);

  nh.initNode();
  //nh.advertise(roll_pub);
  //nh.advertise(pitch_pub);
  nh.advertise(yaw_pub);
  nh.advertise(pubr);
  nh.advertise(publ);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Serial.begin(57600);
}

void loop()
{
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
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  roll  += (GyroX - froll) * dt;
  pitch += (GyroY - fpitch) * dt;
  yaw   += (GyroZ - fyaw) * dt;
  if (yaw > 360)
  {
    yaw -= 360;
  }
  else if (yaw < -360)
  {
    yaw += 360;
  }

  //roll_msg.data = roll;
  //pitch_msg.data = pitch;
  yaw_msg.data      = yaw;

  count_wr_msg.data = pulse_r;
  count_wr_msg.data = pulse_l;

  // roll_pub.publish(&roll_msg);
  // pitch_pub.publish(&pitch_msg);
  yaw_pub.publish(&yaw_msg);
  pubr.publish(&count_wr_msg);
  publ.publish(&count_wl_msg);

  //Serial.print("X : "); Serial.print(roll);
  //Serial.print(", Y : "); Serial.print(pitch);
  //Serial.print(", Z : "); Serial.print(yaw);
  //Serial.println(" ");
  currentTime = millis();
  elapsedTime = currentTime - previousTime;
  dt = elapsedTime / 1000;

  nh.spinOnce();
}

void filter_gyro()
{
  Serial.println("Start probe filter");
  delay(500);
  for (int x = 0; x < 10; x++)
  {
    Serial.print("*");
    delay(100);
  }
  Serial.println("*");

  for (int x = 0; x < 1000; x++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    froll += GyroX;
    fpitch += GyroY;
    fyaw += GyroZ;
  }

  froll = froll / 1000;
  fpitch = fpitch / 1000;
  fyaw = fyaw / 1000;
  Serial.println("Probe filter done!");
  Serial.print("X filter : "); Serial.println(froll);
  Serial.print("Y filter : "); Serial.println(fpitch);
  Serial.print("Z filter : "); Serial.println(fyaw);
}