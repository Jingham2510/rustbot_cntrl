"""
Define and generates the left and right markers for the 6-axis robot and camera calibration process
"""

import cv2 as cv
import matplotlib.pyplot as plt

# The dictionary of choice
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)

marker_savepath = "..\\..\\resources\\cam_calibration\\"


# Define and create the markers
mark_id = 0
mark_size = 400
genned_marker = cv.aruco.generateImageMarker(aruco_dict, mark_id, mark_size)
cv.imwrite(marker_savepath + "left_marker.png", genned_marker)
plt.imshow(genned_marker, cmap="gray", interpolation="nearest")
plt.show()

mark_id = 1
mark_size = 400
genned_marker = cv.aruco.generateImageMarker(aruco_dict, mark_id, mark_size)
cv.imwrite(marker_savepath + "right_marker.png", genned_marker)
plt.imshow(genned_marker, cmap="gray", interpolation="nearest")
plt.show()
