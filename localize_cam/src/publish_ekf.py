#!/usr/bin/python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

flag = 0
raw_ekf = Point(0, 0, 0)
pose_pub = rospy.Publisher("ekf_pose", PointStamped, queue_size=10)

def callback(ekf_point):
    global flag
    global raw_ekf

    if flag == 0:
        flag = 1
        raw_ekf = ekf_point

if __name__=="__main__":
    rospy.init_node("ekf_flag_publisher", anonymous=True)
    print("EKF pose with flag is being published to the topic /ekf_pose ...")
    
    pose_sub = rospy.Subscriber("no_flag_ekf_pose", Point, callback)  # Subscribe outside the loop

    while not rospy.is_shutdown():
        pose_with_flag = PointStamped()
        pose_with_flag.header = Header()
        pose_with_flag.quaternion = Point(raw_ekf.x, raw_ekf.y, flag)
        pose_pub.publish(pose_with_flag)

        if flag == 1:
            print(flag)

            flag = 0