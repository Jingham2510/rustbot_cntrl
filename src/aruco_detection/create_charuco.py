"""
Define and generates the left and right markers for the 6-axis robot and camera calibration process
"""

import cv2 as cv
import matplotlib.pyplot as plt

# The dictionary of choice
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

marker_savepath = "..\\..\\resources\\cam_calibration\\"
title_list = ["top_left", "top_right", "bottom_left", "bottom_right"]

for i in range(4):
    aruco_dict.bytesList = aruco_dict.bytesList[i * 25 :, :, :]
    charuco = cv.aruco.CharucoBoard((10, 7), 1, 0.7, aruco_dict)
    charuco_image = charuco.generateImage((2000, 2000), None, 0, 1)

    cv.imwrite(marker_savepath + title_list[i] + "_board.tiff", charuco_image)
    cv.imshow("charuco", charuco_image)
    cv.waitKey(0)
