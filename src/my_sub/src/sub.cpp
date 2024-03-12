#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <math.h>

float yaw   = 0.0;
int pulse_r = 0;
int pulse_l = 0;
const int R = 0.08;
float x     = 0.0;
float y     = 0.0;
float the_ta = 0.0;
void yawCallback(const std_msgs::Float32::ConstPtr& msg)
{
  yaw     = msg->data;
}

void xungrCallback(const std_msgs::Int32::ConstPtr& msg)
{
  pulse_r = msg->data;
}

void xunglCallback(const std_msgs::Int32::ConstPtr& msg)
{
  pulse_l = msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber_node");
  ros::NodeHandle nh;

  ros::Subscriber yawSub     = nh.subscribe("yaw_data", 10, yawCallback);
  ros::Subscriber pulse_rSub = nh.subscribe("pulse_r_count", 10, xungrCallback);
  ros::Subscriber pulse_lSub = nh.subscribe("pulse_l_count", 10, xunglCallback);
  
  ros::Publisher odometry    = nh.advertise<geometry_msgs::Point>("odometry", 10);

  ros::Rate rate(10);  // Set pace for loop

  while (ros::ok())
  {
    //Odometry
    float d_center = ((pulse_r+pulse_r)*0.5024)/200;
    the_ta  = yaw;
    x      += d_center * cos(the_ta);
    y      += d_center * sin(the_ta);
    //ROS_INFO("Yaw: %f, Xungr: %d, Xungl: %d", yaw, xungr, xungl);

    //Publish x,y,the_ta
    geometry_msgs::Point msg;
    msg.x = x;
    msg.y = y;
    msg.z = the_ta;
    odometry.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
