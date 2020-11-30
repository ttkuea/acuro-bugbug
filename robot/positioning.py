import math
import threading
import time
from collections import defaultdict
from pprint import pprint

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
from flask import Flask, jsonify

from robot.robot import Robot

real_world_pos = [0, 0, 0]
real_rotation = None
despoint=[0,0]
startpoint=[0,0]

plot = np.zeros((480, 640, 3), np.uint8)
final_plot = np.zeros((480, 640, 3), np.uint8)
final_display = np.zeros((480, 640, 3), np.uint8)
dis_threshold = 0.05

def distance(x1,y1,x2,y2):
    return (x1-x2) * (x1-x2) + (y1-y2) * (y1-y2)

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
        [2.05, -2.035],   # 0
        [0.0, 0.0],       # 1
        [-2.044, 2.027],  # 2
        [2.066, 1.442],   # 3
    ]

    rot_offsets = [
        0,                              # 0
        0,                              # 1
        -90,                            # 2
        -90,                            # 3
    ]

    circle_color = [
        (255, 255, 255),                # 0
        (0, 0, 255),                    # 1
        (255, 255, 0),                  # 2
        (255, 0, 255),                  # 3
    ]

    cam_pos = None

    real_world_pos = [0, 0, 0]
    real_rotation = None
    notfound=0

    while True:
        # world plot
        plot = np.zeros((480, 640, 3), np.uint8)
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
                real_rotation = np.rad2deg(np.arctan2(rs[0][1], rs[1][1])) + rot_offsets[ids[i]]
                real_rotation += (real_rotation < -180) * 360 - (
                    real_rotation > 180
                ) * 360
                cam_pos, q = get_worldPos_from_aruco(tvecs[i], rs)

                if ids[i] in [2,3]:
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
                

                if distance(real_world_pos[0], real_world_pos[1], cam_pos[0], cam_pos[1]) < dis_threshold or notfound > 50:
                    tmp_real_world_pos[0] += cam_pos[0]
                    tmp_real_world_pos[1] += cam_pos[1]
                    tmp_real_world_pos[2] += 1

            if tmp_real_world_pos[2] > 0:
                notfound=0
                real_world_pos[0] = tmp_real_world_pos[0] / tmp_real_world_pos[2]
                real_world_pos[1] = tmp_real_world_pos[1] / tmp_real_world_pos[2]
            else:
                notfound+=1

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
                        x_center + int(real_world_pos[0] * zoom - radius * np.sin((real_rotation + 17) * np.pi / 180)),
                        y_center + int(real_world_pos[1] * zoom - radius * np.cos((real_rotation + 17) * np.pi / 180)),
                    ),
                    (255, 255, 255),
                    1,
                )
            except Exception as e:
                print('robot', e)
        else:
            display = frame
        final_plot = plot.copy()
        final_display = display.copy()


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
    
    def setTarget(self, dest):
        global despoint
        despoint = dest
    
    def setStart(self, start):
        global startpoint
        print(start)
        startpoint = start
