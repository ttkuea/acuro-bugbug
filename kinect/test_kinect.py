# import the necessary modules
import pprint

import cv2
import freenect
import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


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

    xplot = []
    yplot = []

    for i in range(array.shape[1]):  # i = width == col
        for j in range(array.shape[0]):  # j = height == row
            xx = (i - w / 2) * z[j, i] / focal_x
            yy = (j - h / 2) * z[j, i] / focal_y + 0.3
            if z[j, i] > 0 and z[j, i] < 4 and yy < 0.32 and yy > 0.05:
                xplot.append(xx)
                yplot.append(z[j, i])
    print(len(xplot))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(xplot, yplot)
    ax.set_xlabel("X Label")
    ax.set_ylabel("Z Label")
    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.show()


ctx = freenect.init()
if __name__ == "__main__":
    while 1:
        # get a frame from RGB camera
        # frame = get_video()
        # get a frame from depth sensor
        depth = get_depth()
        # display RGB image
        # cv2.imshow('RGB image',frame)
        # display depth image
        depth = cv2.circle(depth, (480, 360), 3, (0, 255, 0), -1)

        cv2.imshow("Depth image", depth)

        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
        elif k == ord("a"):
            array = get_real_depth()

            get_real_obs_pos(array)
            # print(pos)

            # array, = freenect.get_depth()
            # rawDisparity = array[240][320]
            # real_dist = get_real_dist(array)
            # print("shape", real_dist.shape)
            # print(real_dist.shape)
            # print(real_dist[240][320])
            # print(array.shape)
            # print(array[0])
    cv2.destroyAllWindows()
