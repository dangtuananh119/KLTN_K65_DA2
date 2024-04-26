#!/usr/bin/python3
import rospy
import pyrealsense2 as rs
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ImagePublisher():
    def __init__(self) -> None:
        self.image_pub = rospy.Publisher("localize_img", Image, queue_size=10)
        self.bridge = CvBridge()

        '''logitech'''
        self.capture = cv2.VideoCapture('/dev/video2')
        
        imgsz = (960, 544)
        if type(imgsz) is tuple:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, imgsz[0])
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, imgsz[1])

        # '''realsense'''
        # self.pipe = rs.pipeline()
        # self.cfg = rs.config()

        # self.cfg.enable_stream(rs.stream.infrared)
        # self.cfg.enable_stream(rs.stream.color, rs.format.bgr8, 30)
        # self.cfg.enable_stream(rs.stream.depth, rs.format.z16, 30)

        # self.pipe.start(self.cfg)

    def logitech_publish_callback(self, event):
        ret, img = self.capture.read()
        if not ret:
            rospy.ERROR("Could not grab a frame!")
            return
        try:
            self.img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_pub.publish(self.img_msg)
        except CvBridgeError as error:
            print(error)


    def realsense_callback(self, event):
        frame = self.pipe.wait_for_frames()

        if frame is None:
            rospy.ERROR("Could not grab a frame!")
            return
        try:
            # ir_frame = frame.first(rs.stream.infrared)
            # depth_frame = frame.get_depth_frame()
            color_frame = frame.get_color_frame()

            # ir_image = np.asanyarray(ir_frame.get_data())
            # depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha = 0.5), cv2.COLORMAP_JET)

            img_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            self.image_pub.publish(img_msg)
        except CvBridgeError as error:
            print(error)

    # pipe.stop()
def publish_image():
    ip = ImagePublisher()
    
    rate = 30.0    # Hz
    
    rospy.Timer(rospy.Duration(1.0/rate), ip.logitech_publish_callback)
    # rospy.Timer(rospy.Duration(1.0/100.0), ip.realsense_callback)
    rospy.spin()
        

if __name__=="__main__":
    rospy.init_node("localize_cam_publisher", anonymous=True)
    print("Image is being published to the topic /localize_img ...")
    publish_image()