#include <ros/ros.h>
#include <std_msgs/Int32.h>

void pulseCountCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("Received pulse count: %d", msg->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/pulse_count", 10, pulseCountCallback);

  ros::spin();

  return 0;
}
