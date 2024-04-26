#!/usr/bin/python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# def display(img, bbox):
#     n = len(bbox)
#     for j in range(n):
#         point1 = tuple(bbox[j][0])
#         point2 = tuple(bbox[(j+1) % n][0])
#         cv2.line(img, point1, point2, (255,0,0), 3)
#     cv2.imshow('ROS Image Subscriber', img)
#     cv2.waitKey(10)
def display(img, bbox):
    for points in bbox:
        # Convert the points to tuple and then to integers
        points = tuple(map(lambda x: tuple(map(int, x)), points))

        # Draw lines to form the quadrilateral
        for j in range(4):
            cv2.line(img, points[j], points[(j + 1) % 4], (255, 0, 0), 3)

    # Display the image
    cv2.imshow('ROS Image Subscriber', img)
    cv2.waitKey(10)


def callback(image_msg):
    """
    This function is called to handle the subscriber messages

    Args:
        img_msg (Image): message type Image from
    """

    try: 
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        cv2.imshow('ROS Image Subscriber', cv_image)
        cv2.waitKey(10)

    except CvBridgeError as error:
        print(error)



def QRcallback(image_msg):
    try: 
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        qrDecoder = cv2.QRCodeDetector()

        data, bbox, rectified_img = qrDecoder.detectAndDecode(cv_image)

        if len(data) > 0:
            print("Decoded data: {}".format(data))
            display(cv_image, bbox)
            rectified_img = np.uint8(rectified_img)
            cv2.imshow("Rectified image_msg", rectified_img)
        else:
            print("QR code not detected")
            cv2.imshow("Results", cv_image)
            cv2.waitKey(10)
    except CvBridgeError as error:
        print(error)


if __name__ == "__main__":
    bridge = CvBridge()
    rospy.init_node("image_subscirber", anonymous=True)
    print("Subscribe images from topic /image_raw ...")

    image_subscriber = rospy.Subscriber("image_raw", Image, QRcallback)

    try:
        # spin() simply keeps python from exiting until this node isnstopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

