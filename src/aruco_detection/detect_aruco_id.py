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

    if ids is not None:
        sys.stdout.write(f"ID_COUNT:{len(ids)}\n")
        # Pipe every ID visible
        id_count = 0
        for id in ids:
            # Send the data to the program pipes
            sys.stdout.write(f"MARK:[{str(ids[id_count])}]\n")
            sys.stdout.write(f"CORN:[{str(corners[id_count])}]\n")
            # Increase the ID count
            id_count += 1

    else:
        sys.stdout.write(":NONE")


if __name__ == "__main__":
    main()
