import cv2
import numpy as np
import os

# 21 ArUco dictionaries: DICT_NxN_M
# NxN: 2D bit size
# M: number of unique values
# 1. Don't take more than what you need
# 2. Look at input resolution size
# 3. Consider the inter-maker distance (affecting the error correction ability)

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

dirname = 'catkin_ws/src/code_gen/curr_code_dict'
if not os.path.isdir(dirname) :
    os.mkdir(dirname)  # make sure the directory exists

total_ID = 50
dictionary = ARUCO_DICT["DICT_4X4_50"]
arucoDict = cv2.aruco.getPredefinedDictionary(dictionary)

for id in range(0, total_ID):
    tag = np.zeros((300, 300, 1), dtype="uint8")
    cv2.aruco.generateImageMarker(arucoDict, id, 300, tag, 1)
    output_path = dirname + "/" + str(id) + ".png"
    cv2.imwrite(output_path, tag)

# # write the generated ArUco tag to disk then display on screen
# cv2.imshow("ArUco Tag", tag)
# cv2.waitKey(0)

