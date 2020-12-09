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


def get_real_obs_pos(array):
    w = 640
    h = 480
    z = get_real_dist(array)
    x = np.zeros(array.shape)
    y = np.zeros(array.shape)

    focal_y = 480 / (2 * np.tan(43 / 2))
    focal_x = 640 / (2 * np.tan(57 / 2))

    # zi = [[0, np.inf]] * 640  # x, y, z

    xplot = []
    yplot = []

    for i in range(array.shape[1]):  # i = width == col
        for j in range(array.shape[0]):  # j = height == row
            xx = z[j, i] * (i - w / 2) / focal_x * 2
            yy = z[j, i] * (j - h / 2) / focal_y + 0.3
            if (
                z[j, i] > 0
                and z[j, i] < 5
                and yy < 0.27
                and yy > 0.05
                # and z[j, i] < zi[i][1]
            ):
                xplot.append(xx)
                yplot.append(z[j, i])
                # zi[i] = [xx, z[j, i]]

    # for p in zi:
    #     if p[1] is not np.inf:
    #         xplot.append(p[0])
    #         yplot.append(p[1])

    return np.stack((np.array(xplot), np.array(yplot)), axis=-1)
