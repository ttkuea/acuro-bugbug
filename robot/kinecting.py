# import the necessary modules
import cv2
import freenect
import numpy as np


# function to get RGB image from kinect
def get_video():
    array, _ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


# function to get depth image from kinect
def get_depth():
    array, _ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array


def get_real_depth():
    array, _ = freenect.sync_get_depth()
    return array


def get_real_dist(array):
    # THIS IS FUCKING MAGIC WE DON'T KNOW WHAT IT IS
    return 0.1236 * np.tan(array / 2842.5 + 1.1863)


w = 640
h = 480

focal_y = 480 / (2 * np.tan(43 / 2))
focal_x = 640 / (2 * np.tan(57 / 2))


def point_transform(Z, i, j):
    z = Z[j, i]
    x = z * (i - w / 2) / focal_x * 2
    y = z * (j - h / 2) / focal_y + 0.3

    return [x, y, z]


def get_real_obs_pos(array):
    Z = get_real_dist(array)

    indices = np.floor(
        np.random.rand(1000, 2) @ np.array([[array.shape[0], 0], [0, array.shape[1]]])
    ).astype(int)

    # indices = np.ndindex(*Z.shape)

    points = np.array([point_transform(Z, i, j) for j, i in indices])

    rows = np.where(
        (points[:, 1] < 0.27)
        & (points[:, 1] > 0.05)
        & (points[:, 2] > 0.5)
        & (points[:, 2] < 6)
    )

    choosen = points[rows]

    return np.stack([choosen[:, 0], choosen[:, 2]], axis=1)
