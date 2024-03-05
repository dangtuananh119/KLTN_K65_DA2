#include <ros/ros.h>
#include <std_msgs/Float32.h>

void rollCallback(const std_msgs::Float32::ConstPtr& msg)
{
  float roll = msg->data;
  ROS_INFO("X: %f", roll);
}

void pitchCallback(const std_msgs::Float32::ConstPtr& msg)
{
  float pitch = msg->data;
  ROS_INFO("Y: %f", pitch);
}

void yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
  float yaw = msg->data;
  ROS_INFO("Z: %f", yaw);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_angle");
  ros::NodeHandle nh;

  //ros::Subscriber rollSub = nh.subscribe("roll_data", 10, rollCallback);
  //ros::Subscriber pitchSub = nh.subscribe("pitch_data", 10, pitchCallback);
  ros::Subscriber yawSub = nh.subscribe("yaw_data", 10, yawCallback);

  ros::spin();

  return 0;
}