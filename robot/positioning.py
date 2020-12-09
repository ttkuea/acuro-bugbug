import math
import threading
import time
from collections import defaultdict
from pprint import pprint

import cv2
import cv2.aruco as aruco
import freenect
import numpy as np
import yaml
from flask import Flask, jsonify

from robot.robot import Robot

from .kinecting import get_real_depth, get_real_dist, get_real_obs_pos
from .occupancy_grid import OccupancyGrid

real_world_pos = [0, 0, 0]
real_rotation = None
despoint = [0, 0]
startpoint = [0, 0]

plot = np.zeros((480, 640, 3), np.uint8)
final_plot = np.zeros((480, 640, 3), np.uint8)
final_display = np.zeros((480, 640, 3), np.uint8)
final_occupancy = np.zeros((200, 200, 1), np.uint8)
dis_threshold = 0.05


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


def positionThread():
    global real_world_pos
    global real_rotation
    global final_plot
    global final_display
    global final_occupancy

    ctx = freenect.init()

    plot = np.zeros((480, 640, 3), np.uint8)
    display = np.zeros((480, 640, 3), np.uint8)

    print(cv2.__version__)

    cap = cv2.VideoCapture(2)
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

    real_world_pos = [0, 0, 0]
    real_rotation = None
    notfound = 0
    ready = False

    # Kinect
    resolution = 0.1525
    occupancy_grid = OccupancyGrid(shape=(40, 40), resolution=resolution)
    occupancy_grid.min_treshold = -50
    occupancy_grid.max_treshold = 50
    min_coordinate = resolution * 20

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
                real_rotation = (
                    np.rad2deg(np.arctan2(rs[0][1], rs[1][1])) + rot_offsets[ids[i]]
                )
                real_rotation += (real_rotation < -180) * 360 - (
                    real_rotation > 180
                ) * 360
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
                    ready = True
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
                            - radius * np.sin((real_rotation + 17) * np.pi / 180)
                        ),
                        y_center
                        + int(
                            real_world_pos[1] * zoom
                            - radius * np.cos((real_rotation + 17) * np.pi / 180)
                        ),
                    ),
                    (255, 255, 255),
                    1,
                )
            except Exception as e:
                print("robot", e)
        else:
            display = frame
        # Kinect data
        array = get_real_depth()
        npdata = get_real_obs_pos(array)
        # print(npdata.shape)
        npdata = npdata[np.random.randint(npdata.shape[0], size=100), :]
        npdata = sorted(npdata, key=lambda x: -x[1])
        for point in npdata:
            r_sin = np.sin((real_rotation + 17) * np.pi / 180)
            r_cos = np.cos((real_rotation + 17) * np.pi / 180)
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
            # occupancy_grid.updateOccupy((3, 3), (0, 0))
            # occupancy_grid.updateOccupy((3, 3), (6, 6))
            # occupancy_grid.updateOccupy((3, 3), (6, 0))
            # occupancy_grid.updateOccupy((3, 3), (0, 6))

        # print(occupancy_grid.grid)
        final_plot = plot.copy()
        final_display = display.copy()
        occupancy_range = occupancy_grid.max_treshold - occupancy_grid.min_treshold
        final_occupancy = (
            (occupancy_grid.grid - occupancy_grid.min_treshold) / occupancy_range * 255
        ).astype(np.uint8)


# -----------------------------------------------------------

# -----------------------------------------------------------


class GlobalData:
    def __init__(self):
        t1 = threading.Thread(target=positionThread)
        t1.start()

    def getPosition(self):
        return real_world_pos

    def getRotation(self):
        return real_rotation

    def getPlot(self):
        return final_plot

    def getDisplay(self):
        return final_display

    def getOccupancy(self):
        return final_occupancy

    def setTarget(self, dest):
        global despoint
        despoint = dest

    def setStart(self, start):
        global startpoint
        print(start)
        startpoint = start
