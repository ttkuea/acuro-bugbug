import cv2
import freenect
import numpy as np


KINECT_MAX_DEPTH = 1024

KINECT_MIN_DISTANCE = -10
KINECT_SCALE_FACTOR = 0.0021

class Kinect:

    def __init__(self):
        # self.depthLU = [ for i in range(KINECT_MAX_DEPTH)]
        pass

    def init(self):
        # try:
        #     freenect.init()
        # except:
        #     print('ERROR: can\'t initialize freenect')
        pass

    def getPoint(idx, z, w, h):
        i = idx % w
        j = idx // w
        x = (i - w / 2) * (z + KINECT_MIN_DISTANCE) * KINECT_SCALE_FACTOR
        y = (j - h / 2) * (z + KINECT_MIN_DISTANCE) * KINECT_SCALE_FACTOR
        return x, y, z

    def getPoints(self):
        depth, _ = freenect.sync_get_depth()
        # print(depth)
        self.h, self.w = depth.shape

        def rawasfuck(i):
            return 0.1236 * np.tan(i / 2842.5 + 1.1863)

        depth_flat = rawasfuck(depth.flatten('C'))
        idx = np.array(range(len(depth_flat)))
        points = Kinect.getPoint(idx, depth_flat, self.w, self.h)
        for p in points:
            print(p.shape)

        print(depth[:5, :2], depth[:5], depth[self.w: self.w + 5])

        return points, depth

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    # print(freenect.sync_get_depth())

    kinect = Kinect()
    kinect.init()

    points, d = kinect.getPoints()
    X, Y, Z = points

    cv2.imshow('d', d.astype(np.uint8))
    ax.scatter(- X, Z, Y, s=1, c=-Z)

    fig1, ax1 = plt.subplots()
    # ax1.scatter(- X, Y, s=1, c=-Z)
    ax1.scatter( Z, Y, s=1, c=-Z)
    # ax1.scatter(-X, Z, s=1, c=-Z)
    plt.show()