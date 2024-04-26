#!/usr/bin/python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

import json 

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def extract_camera_pose(input_img):
    output_matrix = np.array([[0, 0, 0, 0],
                              [0, 0, 0, 0],
                              [0, 0, 0, 0],
                              [0, 0, 0, 1]], dtype=np.float32)
    marker_ID = -1
    
    (corners, ids, _) = cv2.aruco.detectMarkers(input_img, arucoDict, parameters=arucoParams) # corners, ids, rejected 
    if len(corners) > 0:
        # flatten the ArUco ID list:
        ids = ids.flatten()
        
        r_vec, t_vec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SZIE, cam_mat, dist_coef)
        total_markers = range(0, ids.size)
        dist = np.zeros(ids.size)

        # loop over the detected ArUco corners
        for (id, i) in zip(ids, total_markers):
            translation = t_vec[i][0]
            rotation = r_vec[i][0]
            x = t_vec[i][0][0]
            y = t_vec[i][0][1]
            z = t_vec[i][0][2]          
            
            dist[i] = np.sqrt(x**2 + y**2 + z**2)

        # choose the closest marker if many are detected
        i_min = np.argmin(dist)
        translation = t_vec[i_min][0]
        rotation = r_vec[i_min][0]
        marker_ID = ids[i_min]

        rotation_mat, _ = cv2.Rodrigues(rotation)       # orientation_mat, jacobian
        
        # transformation matrix
        # print (rotation_mat)
        # print (translation)
        rotation_mat = rotation_mat.T
        translation = -1 * rotation_mat @ translation.reshape((3))

        output_matrix[0:3, 0:3] = rotation_mat
        output_matrix[0:3, 3] = translation
        # output_matrix = np.linalg.inv(output_matrix)

        # print(output_matrix)
        # show image to visualize
        output_img = input_img.copy()

        # the pose of the marker with repsect to the camera
        output_img = cv2.aruco.drawDetectedMarkers(output_img, corners=corners, ids=ids, borderColor=(0, 255, 0))
        # output_img = cv2.drawFrameAxes(output_img, cam_mat, dist_coef, r_vec, t_vec, 3, 2)
        cv2.imshow("window", output_img)
        cv2.waitKey(10)
    else:
        cv2.imshow("window", input_img)
        cv2.waitKey(10)

    return marker_ID, output_matrix

def transform_frames(landmark_ID, T_cam_barcode):
    # each homogenous transformation matrix (HTM) is 4x4

    # HTM from global frame to landmark frame
    # if ID is even, robot is moving forward, otherwise its moving backward
    # starting point at ID = 0
    # camera is on the left side of the robot
    # matrix corresponding to each ID is predefined

    T_barcode_global = np.array(posList[landmark_ID])

    # HTM from camera frame to moving frame
    # this matrix is predefined baseed on robot construction
    T_moving_cam = np.array([[1, 0,  0, -15],
                             [0, 0, -1,  5],   # representative for pos of camera to robot centers
                             [0, 1,  0, -14.5],   # now assume its 10cm to the left of the center, meaning robot center has z=-10 in cam coord
                             [0, 0 , 0,  1]])
    
    # HTM from camera to barcode
    # this matrix is obtained through esitmatePoseSingleMarker
    # using rVec and Rodrigue transform to obtain rotation matrix (3x3)
    # its already calculated previous to this step

    # HTM from moving to global
    # now compute successive multiplication of the above HTMs
    T_moving_global = T_barcode_global @ T_cam_barcode 
    T_moving_global = T_moving_global @ T_moving_cam


    # now obtained position vector and orientation of robot
    position = T_moving_global[0:3, 3]
    orientation_matrix = T_moving_global[0:3, 0:3]
    # orientation is the angle between predefined x-axis of robot to global
    orientation =  np.arctan(orientation_matrix[1][0] / orientation_matrix[1][1])
    return position, orientation

def pub1_callback(event):
    global marker_ID
    if marker_ID >= 0:
        print(marker_ID, mes_Z.point.x, mes_Z.point.y, mes_Z.point.z)
        point_pub.publish(mes_Z)

def pub2_callback(event):
    global marker_ID

    marker_msg.header = Header()
    marker_msg.point = Point(marker_ID, 0, 0)
    marker_pub.publish(marker_msg)

def sub_callback(image_msg):
    global marker_ID
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg)
        
        marker_ID, T_cam_barcode = extract_camera_pose(cv_image)
        if marker_ID >= 0:  # -1 = not detected
            position, orientation = transform_frames(marker_ID, T_cam_barcode)
            mes_Z.header = Header()
            mes_Z.point = Point(x=position[0]/100, y=position[1]/100, z=orientation)

    except CvBridgeError as error:
        print(error)

if __name__=="__main__":
    # read json for map data
    with open("/home/trungle/catkin_ws/src/localize_cam/map.json", "r") as file:
        map_data = json.load(file)

    posList = map_data[0]['pos']
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[map_data[0]['barcode_type']])
    arucoParams = cv2.aruco.DetectorParameters()

    # read csv for calibration data
    calib_data_path = "/home/trungle/catkin_ws/src/calibration/logitech/calib_savez.npz"
    calib_data = np.load(calib_data_path)
    cam_mat = calib_data["cameraMatrix"]
    dist_coef = calib_data["distCoef"]
    rvecs = calib_data["rVectors"]  # dont need
    tvecs = calib_data["tVectors"]  # dont need

    MARKER_SZIE = map_data[0]['marker_size'] # centimeters

    # node activity
    # previous_pose = np.array([0, 0, 0])
    rate = 2.0
    marker_ID = -1
    bridge = CvBridge()
    rospy.init_node("localize_image_subscriber", anonymous=True)
    marker_msg = PointStamped()
    mes_Z = PointStamped()
    print("Measured pose is being published to the topic /measured_pose ...")
    point_pub = rospy.Publisher("measured_pose", PointStamped, queue_size=10)

    print("MarkerID is being published to the topic /marker_id ...")
    marker_pub = rospy.Publisher("marker_id", PointStamped, queue_size=10)

    print("Subscribe images from topic /localize_img ...")

    image_subscriber = rospy.Subscriber("localize_img", Image, sub_callback)

    rospy.Timer(rospy.Duration(1/rate), pub1_callback)
    rospy.Timer(rospy.Duration(1/30.0), pub2_callback)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown!")
