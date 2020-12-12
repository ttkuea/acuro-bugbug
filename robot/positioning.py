import ctypes
import math
import threading
import time
from collections import defaultdict
from multiprocessing import Array, Manager, Process
from multiprocessing.sharedctypes import RawArray
from pprint import pprint

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

from robot.robot import Robot


def draw_grid(image):
    rows = 480
    cols = 640

    color = (50, 50, 50)

    step = 10
    x = np.linspace(start=0, stop=cols, num=cols // step)
    y = np.linspace(start=0, stop=rows, num=rows // step)

    v_xy = []
    h_xy = []
    for i in range(cols // step):
        v_xy.append([int(x[i]), 0, int(x[i]), rows - 1])
    for i in range(rows // step):
        h_xy.append([0, int(y[i]), cols - 1, int(y[i])])

    for i in range(cols // step):
        [x1, y1, x2, y2] = v_xy[i]
        image = cv2.line(image, (x1, y1), (x2, y2), color, 1)
    for i in range(rows // step):
        [x1_, y1_, x2_, y2_] = h_xy[i]
        image = cv2.line(image, (x1_, y1_), (x2_, y2_), color, 1)
    return image


def distance(x1, y1, x2, y2):
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)


def get_worldPos_from_aruco(tvec, dstl):
    extristics = np.matrix(
        [
            [dstl[0][0], dstl[0][1], 0, tvec[0][0]],
            [dstl[1][0], dstl[1][1], 0, tvec[0][1]],
            [0, 0, 1, tvec[0][2]],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    extristics_I = extristics.I  # inverse matrix
    worldPos = [
        extristics_I[0, 3],
        extristics_I[1, 3],
        extristics_I[2, 3],
    ]

    return np.array(worldPos), 1


def positionProcess(
    real_world_pos,
    real_rotation,
    final_plot,
    final_display,
    despoint,
    startpoint,
    ready,
):
    dis_threshold = 0.05
    # These will need to be shared
    # global real_world_pos
    # global real_rotation
    # global final_plot
    # global final_display
    # global final_occupancy

    plot = np.zeros((480, 640, 3), np.uint8)
    display = np.zeros((720, 1280, 3), np.uint8)

    print(cv2.__version__)

    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)
    cap.set(4, 720)
    # cap.set(5, 60)
    with open("calibration_finala.yaml") as f:
        loadeddict = yaml.load(f)
        mtx = loadeddict.get("camera_matrix")
        dist = loadeddict.get("dist_coeff")
        mtx = np.array(mtx)
        dist = np.array(dist)

    pos_offsets = [
        [2.05, -2.035],  # 0
        [0.0, 0.0],  # 1
        [-2.044, 2.027],  # 2
        [2.066, 1.442],  # 3
    ]

    rot_offsets = [
        0,  # 0
        0,  # 1
        -90,  # 2
        -90,  # 3
    ]

    circle_color = [
        (255, 255, 255),  # 0
        (0, 0, 255),  # 1
        (255, 255, 0),  # 2
        (255, 0, 255),  # 3
    ]

    cam_pos = None
    notfound = 0
    ready.value = False

    # Kinect
    # resolution = 0.1525
    # occupancy_grid = OccupancyGrid(shape=(40, 40), resolution=resolution)
    # occupancy_grid.min_treshold = -50
    # occupancy_grid.max_treshold = 50
    # min_coordinate = resolution * 20

    while True:
        # world plot
        plot = np.zeros((480, 640, 3), np.uint8)
        plot = draw_grid(plot)
        x_center = plot.shape[1] // 2
        y_center = plot.shape[0] // 2
        zoom = 40
        plot = cv2.circle(plot, (320, 240), 5, (0, 255, 0), 1)

        # plot starting point and destination
        plot = cv2.circle(
            plot,
            (
                x_center + int(despoint[0] * zoom),
                y_center + int(despoint[1] * zoom),
            ),
            5,
            (0, 0, 255),
            -1,
        )

        plot = cv2.circle(
            plot,
            (
                x_center + int(startpoint[0] * zoom),
                y_center + int(startpoint[1] * zoom),
            ),
            5,
            (0, 255, 0),
            -1,
        )
        plot = cv2.line(
            plot,
            (
                x_center + int(despoint[0] * zoom),
                y_center + int(despoint[1] * zoom),
            ),
            (
                x_center + int(startpoint[0] * zoom),
                y_center + int(startpoint[1] * zoom),
            ),
            (0, 255, 255),
            1,
        )

        ret, frame = cap.read()

        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict)
        if np.all(ids != None):
            ids = [id[0] for id in ids]
            display = aruco.drawDetectedMarkers(frame, corners)
            size_of_marker = 0.3  # side lenght of the marker in meter
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, size_of_marker, mtx, dist
            )

            tmp_real_world_pos = [0, 0, 0]  # x,y,count
            for i in range(len(tvecs)):
                length_of_axis = 0.3

                display = aruco.drawAxis(
                    display, mtx, dist, rvecs[i], tvecs[i], length_of_axis
                )
                rs, _ = cv2.Rodrigues(rvecs[i])
                real_rot = (
                    np.rad2deg(np.arctan2(rs[0][1], rs[1][1])) + rot_offsets[ids[i]]
                )
                real_rot += (real_rot < -180) * 360 - (real_rot > 180) * 360
                real_rotation.value = real_rot
                cam_pos, q = get_worldPos_from_aruco(tvecs[i], rs)

                if ids[i] in [2, 3]:
                    cam_pos[0], cam_pos[1] = -cam_pos[1], cam_pos[0]
                cam_pos[0] += pos_offsets[ids[i]][0]
                cam_pos[1] += pos_offsets[ids[i]][1]
                try:
                    plot = cv2.circle(
                        plot,
                        (
                            x_center + int(cam_pos[0] * zoom),
                            y_center + int(cam_pos[1] * zoom),
                        ),
                        5,
                        circle_color[i],
                        1,
                    )
                except Exception as e:
                    print(i, e)

                if (
                    distance(
                        real_world_pos[0], real_world_pos[1], cam_pos[0], cam_pos[1]
                    )
                    < dis_threshold
                    or notfound > 5
                ):
                    ready.value = True
                    tmp_real_world_pos[0] += cam_pos[0]
                    tmp_real_world_pos[1] += cam_pos[1]
                    tmp_real_world_pos[2] += 1
            if tmp_real_world_pos[2] > 0:
                notfound = 0
                real_world_pos[0] = tmp_real_world_pos[0] / tmp_real_world_pos[2]
                real_world_pos[1] = tmp_real_world_pos[1] / tmp_real_world_pos[2]
            else:
                notfound += 1

            try:
                # plot robot position
                plot = cv2.circle(
                    plot,
                    (
                        x_center + int(real_world_pos[0] * zoom),
                        y_center + int(real_world_pos[1] * zoom),
                    ),
                    1,
                    (255, 255, 255),
                    -1,
                )
                radius = 20
                plot = cv2.circle(
                    plot,
                    (
                        x_center + int(real_world_pos[0] * zoom),
                        y_center + int(real_world_pos[1] * zoom),
                    ),
                    radius,
                    (255, 255, 255),
                    1,
                )
                plot = cv2.line(
                    plot,
                    (
                        x_center + int(real_world_pos[0] * zoom),
                        y_center + int(real_world_pos[1] * zoom),
                    ),
                    (
                        x_center
                        + int(
                            real_world_pos[0] * zoom
                            - radius * np.sin((real_rotation.value + 17) * np.pi / 180)
                        ),
                        y_center
                        + int(
                            real_world_pos[1] * zoom
                            - radius * np.cos((real_rotation.value + 17) * np.pi / 180)
                        ),
                    ),
                    (255, 255, 255),
                    1,
                )
            except Exception as e:
                print("robot", e)
        else:
            display = frame
        """
        # Kinect data
        array = get_real_depth()
        npdata = get_real_obs_pos(array)
        # print(npdata.shape)
        npdata = npdata[np.random.randint(npdata.shape[0], size=100), :]
        npdata = sorted(npdata, key=lambda x: -x[1])
        
        for point in npdata:
            r_sin = np.sin((real_rotation.value + 17) * np.pi / 180)
            r_cos = np.cos((real_rotation.value + 17) * np.pi / 180)
            R = np.array([[r_cos, -r_sin], [r_sin, r_cos]])
            point_rotated = R @ np.reshape(point, (2, 1))
            plot = cv2.circle(
                plot,
                (
                    x_center + int((real_world_pos[0] + point_rotated[0][0]) * zoom),
                    y_center + int((real_world_pos[1] - point_rotated[1][0]) * zoom),
                ),
                1,
                (0, 255, 0),
                -1,
            )
            if ready:
                occupancy_grid.updateOccupy(
                    (
                        real_world_pos[0] + min_coordinate,
                        real_world_pos[1] + min_coordinate,
                    ),
                    (
                        real_world_pos[0] + point_rotated[0][0] + min_coordinate,
                        real_world_pos[1] - point_rotated[1][0] + min_coordinate,
                    ),
                )
        """
        # occupancy_grid.updateOccupy((3, 3), (0, 0))
        # occupancy_grid.updateOccupy((3, 3), (6, 6))
        # occupancy_grid.updateOccupy((3, 3), (6, 0))
        # occupancy_grid.updateOccupy((3, 3), (0, 6))
        # print(occupancy_grid.grid)
        memoryview(final_plot).cast("B")[:] = plot.flatten()[:]
        memoryview(final_display).cast("B")[:] = display.flatten()[:]


# -----------------------------------------------------------
# Kinect Process


def kinect_process(real_world_pos, real_rotation, ready, final_occupancy):
    import freenect

    from .kinecting import get_real_depth, get_real_dist, get_real_obs_pos
    from .occupancy_grid import OccupancyGrid

    # Kinect
    resolution = 0.1525
    occupancy_grid = OccupancyGrid(shape=(40, 40), resolution=resolution)
    occupancy_grid.min_treshold = -50
    occupancy_grid.max_treshold = 50
    min_coordinate = resolution * 20

    ctx = freenect.init()

    while True:
        # Kinect data
        array = get_real_depth()
        npdata = get_real_obs_pos(array)
        if npdata.shape[0] > 0:
            npdata = npdata[np.random.randint(npdata.shape[0], size=200), :]
        npdata = sorted(npdata, key=lambda x: -x[1])

        for point in npdata:
            if ready.value:
                r_sin = np.sin((real_rotation.value + 17) * np.pi / 180)
                r_cos = np.cos((real_rotation.value + 17) * np.pi / 180)
                R = np.array([[r_cos, -r_sin], [r_sin, r_cos]])
                point_rotated = R @ np.reshape(point, (2, 1))
                occupancy_grid.updateOccupy(
                    (
                        real_world_pos[0] + min_coordinate,
                        real_world_pos[1] + min_coordinate,
                    ),
                    (
                        real_world_pos[0] + point_rotated[0][0] + min_coordinate,
                        real_world_pos[1] - point_rotated[1][0] + min_coordinate,
                    ),
                )
        occupancy_range = occupancy_grid.max_treshold - occupancy_grid.min_treshold
        final_occupancies = (
            (occupancy_grid.grid - occupancy_grid.min_treshold) / occupancy_range * 255
        ).astype(np.uint8)
        memoryview(final_occupancy).cast("B")[:] = final_occupancies.flatten()[:]


# -----------------------------------------------------------


class GlobalData:
    def __init__(self):
        manager = Manager()
        self.real_world_pos = manager.list([0, 0, 0])
        self.real_rotation = manager.Value("d", 0.0)
        self.ready = manager.Value(ctypes.c_bool, False)
        self.final_plot = RawArray(ctypes.c_int8, 480 * 640 * 3)
        self.final_display = RawArray(ctypes.c_int8, 720 * 1280 * 3)
        self.final_occupancy = RawArray(ctypes.c_int8, 40 * 40)
        self.despoint = manager.list([0, 0])
        self.startpoint = manager.list([0, 0])
        t1 = Process(
            target=positionProcess,
            args=(
                self.real_world_pos,
                self.real_rotation,
                self.final_plot,
                self.final_display,
                self.despoint,
                self.startpoint,
                self.ready,
            ),
        )
        t1.start()
        t2 = Process(
            target=kinect_process,
            args=(
                self.real_world_pos,
                self.real_rotation,
                self.ready,
                self.final_occupancy,
            ),
        )
        t2.start()

    def getPosition(self):
        return self.real_world_pos

    def getRotation(self):
        return self.real_rotation.value

    def getPlot(self):
        arr = np.frombuffer(self.final_plot, dtype=np.uint8)  # no data copying
        arr = arr.reshape((480, 640, 3))
        return arr

    def getDisplay(self):
        arr = np.frombuffer(self.final_display, dtype=np.uint8)  # no data copying
        arr = arr.reshape((720, 1280, 3))
        return arr

    def getOccupancy(self):
        arr = np.frombuffer(self.final_occupancy, dtype=np.uint8)  # no data copying
        arr = arr.reshape((40, 40))
        return arr

    def setTarget(self, dest):
        self.despoint[0] = dest[0]
        self.despoint[1] = dest[1]

    def setStart(self, start):
        print(start)
        self.startpoint[0] = start[0]
        self.startpoint[1] = start[1]
