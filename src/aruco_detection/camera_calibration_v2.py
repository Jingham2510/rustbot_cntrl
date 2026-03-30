import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
from cv2 import aruco

# ============================================================
# INPUT PARAMETERS (CHANGE)
# ============================================================
board_size = (
    0.86  # m - centre-to-centre/corner-to-corner distance of four aruco markers.
)

marker_coords = {
    # Top left
    0: np.array([0, board_size, 0], dtype=np.float64),
    # Top right
    1: np.array([board_size, board_size, 0], dtype=np.float64),
    # Bottom left (Home)
    2: np.array([0, 0, 0], dtype=np.float64),
    # Bottom Right
    3: np.array([board_size, 0, 0], dtype=np.float64),
}
marker_size = 0.29  # m

# ============================================================
# INPUT PARAMETERS (DON'T CHANGE)
# ============================================================
"""
camr_int = np.asarray(
    [
        [2.54991519e03, 0.00000000e00, 3.64054086e02],
        [0.00000000e00, 2.72042550e03, 2.91611028e02],
        [0.00000000e00, 0.00000000e00, 1.00000000e00],
    ]
)  # Righthand camera intrinsic matrix. - FROM CALIBRATION
"""
camr_int = np.asarray(
    [
        [913.0819, 0.00000000e00, 658.0791],
        [0.00000000e00, 912.98676, 373.58694],
        [0.00000000e00, 0.00000000e00, 1.00000000e00],
    ]
)  # Righthand camera intrinsic matrix. - FROM REALSENSE CAMERA


camr_dist = np.asarray(
    [2.94925139e-01, 5.63870463e01, 1.71032355e-01, 7.65444064e-03, -2.28621415e03]
)  # Righthand camera distortion parameter vector.

"""
caml_int = np.asarray(
    [
        [620.15641364, 0.0, 333.38539868],
        [0.0, 620.87495725, 258.55480798],
        [0.0, 0.0, 1.0],
    ]
)  # Lefthand camera intrinsic matrix. - from calibration
"""

caml_int = np.asarray(
    [
        [967.445, 0.0, 955.357],
        [0.0, 967.445, 529.405],
        [0.0, 0.0, 1.0],
    ]
)  # Lefthand camera intrinsic matrix. - from calibration

caml_dist = np.asarray(
    [1.37376096e-01, -3.03028814e-01, -2.43437907e-04, 2.55729346e-04, -6.73455786e-02]
)  # Lefthand camera distortion parameter vector.

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters()


# ============================================================
# Functions
# ============================================================
def estimate_pose(image, K, dist):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = detector.detectMarkers(gray)

    # aruco_image = cv2.aruco.drawDetectedMarkers(image, corners, ids)
    # plt.imshow(aruco_image)
    # plt.show()

    if ids is None:
        raise ValueError("No ArUco markers detected")

    # Flatten ids
    ids = ids.flatten()

    # Coordinates in world space
    object_points = []
    # Coordinates in image space
    image_points = []

    for i, marker_id in enumerate(ids):
        if marker_id in marker_coords:
            # Use marker center as correspondence
            object_points.append(marker_coords[marker_id])

            # Use detected marker corner center
            c = corners[i][0]
            center = np.mean(c, axis=0)
            image_points.append(center)

    object_points = np.array(object_points, dtype=np.float64)
    image_points = np.array(image_points, dtype=np.float64)

    # print("obj: ", object_points)
    # print("img: ", image_points)

    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(
        object_points, image_points, K, dist, flags=cv2.SOLVEPNP_P3P
    )

    if not success:
        raise RuntimeError("PnP failed")

    return rvec, tvec


def get_extrinsic_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)

    extrinsic = np.eye(4)
    extrinsic[:3, :3] = R
    extrinsic[:3, 3] = tvec.flatten()

    return extrinsic


def invert_extrinsic(extrinsic):
    R = extrinsic[:3, :3]
    t = extrinsic[:3, 3]

    R_inv = R.T
    t_inv = -R_inv @ t

    extrinsic_inv = np.eye(4)
    extrinsic_inv[:3, :3] = R_inv
    extrinsic_inv[:3, 3] = t_inv

    return extrinsic_inv


def camera_to_world(points_cam, extrinsic_inv):
    """
    points_cam: Nx3 in camera coordinates
    extrinsic_inv: 4x4 matrix
    """
    N = points_cam.shape[0]
    homo = np.hstack((points_cam, np.ones((N, 1))))
    world = (extrinsic_inv @ homo.T).T
    return world[:, :3]


def process_camera(image_path, K, dist):
    image = cv2.imread(image_path)

    rvec, tvec = estimate_pose(image, K, dist)
    extrinsic = get_extrinsic_matrix(rvec, tvec)
    extrinsic_inv = invert_extrinsic(extrinsic)

    return {
        "rvec": rvec,
        "tvec": tvec,
        "extrinsic": extrinsic,
        "extrinsic_inv": extrinsic_inv,
    }


caml = process_camera(
    "C:\\Users\\User\\Documents\\Programming\\Projects\\rusbtot_cntrl\\rustbot_cntrl\\appdata\\large_aruco_not_ext.png",
    caml_int,
    caml_dist,
)
camr = process_camera(
    "C:\\Users\\User\\Documents\\Programming\\Projects\\rusbtot_cntrl\\rustbot_cntrl\\appdata\\largo_aruco_joes_cam_ext.png",
    camr_int,
    camr_dist,
)


# left_ext = caml["extrinsic"]
left_inv = caml["extrinsic_inv"]
# print("Left Camera Extrinsic:\n", caml["extrinsic"])
print("Left Camera Inverse Extrinsic:\n", left_inv)

# left_rot = cv2.Rodrigues(left_ext[0:3, 0:3])[0]
# print("Left Camera rot:\n", left_rot)

# left_inv_rot = cv2.Rodrigues(left_inv[0:3, 0:3])[0]
# print("Left Camera inv rot:\n", left_inv_rot)


# right_ext = camr["extrinsic"]
right_inv = camr["extrinsic_inv"]
# print("Right Camera Extrinsic:\n", camr["extrinsic"])
print("Right Camera Inverse Extrinsic:\n", camr["extrinsic_inv"])

# right_rot = cv2.Rodrigues(right_ext[0:3, 0:3])[0]
# print("Right Camera rot:\n", right_rot)

# right_inv_rot = cv2.Rodrigues(right_inv[0:3, 0:3])[0]
# print("Right Camera inv rot:\n", right_inv_rot)
