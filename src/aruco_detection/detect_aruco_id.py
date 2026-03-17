"""
Detects an aruco tag and returns its ID and corners
"""

import sys

import cv2 as cv


def main():

    # Get the system arguments
    args = sys.argv

    print(args)
    image_fp = args[1]

    # Load the image (and give it a border for now)
    image = cv.imread(image_fp)

    # Convert the image to grayscale
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters()

    # Create the ArUco detector
    detector = cv.aruco.ArucoDetector(aruco_dict, parameters)
    # Detect the markers
    corners, ids, rejected = detector.detectMarkers(gray)
    # Print the detected markers
    print("Detected markers:", ids)
    if ids is not None:
        return (ids, corners)


if __name__ == "__main__":
    main()
