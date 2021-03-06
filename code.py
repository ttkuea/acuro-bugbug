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


def get_worldPos_from_aruco(tvec, dstl):
    extristics = np.matrix(
        [
            [dstl[0][0], dstl[0][1], dstl[0][2], tvec[0][0]],
            [dstl[1][0], dstl[1][1], dstl[1][2], tvec[0][1]],
            [dstl[2][0], dstl[2][1], dstl[2][2], tvec[0][2]],
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


print(cv2.__version__)

cap = cv2.VideoCapture(2)
cap.set(3, 640)
cap.set(4, 480)
# cap.set(5, 60)
with open("calibration_final.yaml") as f:
    loadeddict = yaml.load(f)
    mtx = loadeddict.get("camera_matrix")
    dist = loadeddict.get("dist_coeff")
    mtx = np.array(mtx)
    dist = np.array(dist)


data = defaultdict(list)
prev_t = None
prev_r = None
prev_t0 = None
prev_r0 = None
prev_t2 = None
prev_r2 = None
prev_t3 = None
prev_r3 = None

cam_pos = None

rsave = []

r = None
r0 = None
r2 = None
r3 = None
global real_world_pos
global real_rotation
real_world_pos = [0, 0, 0]
real_rotation = None

# --------------FLASKKKKKSKSKSKSKSKSK------------------------
# app = Flask(__name__)
# data = "shit"


# @app.route("/")
# def main():
#     # return jsonify( \
#     #     x = real_world_pos[0], \
#     #     y = real_world_pos[1], \
#     #     z = real_world_pos[2], \
#     #     r = real_rotation, \
#     # )
#     return str(real_world_pos[0])  + "," + str(real_world_pos[1])  + ","+ str(real_world_pos[2])  + "," + np.array2string(real_rotation)


def flaskThread():

    rob = Robot()

    target = [0.0, 0.0]
    state = 0
    while True:
        if state == 0:
            while True:
                nowrotate = 360 - ((real_rotation + 630) % 360)
                diffx = target[0] - real_world_pos[0]
                diffy = target[1] - real_world_pos[1]
                atandeg = np.degrees(np.arctan2(diffy, diffx))
                degreediff = (atandeg - nowrotate + 360) % 360
                print(degreediff, atandeg, nowrotate)
                if abs(nowrotate - atandeg) <= 10:
                    state = 1
                    rob.drivedirect(0, 0)
                    break
                if degreediff > 180:
                    rob.drivedirect(-50, 50)
                else:
                    rob.drivedirect(50, -50)
                time.sleep(0.5)
                rob.drivedirect(0, 0)
                time.sleep(1)
        if state == 1:
            while True:
                nowrotate = 360 - ((real_rotation + 630) % 360)
                diffx = target[0] - real_world_pos[0]
                diffy = target[1] - real_world_pos[1]
                dis = math.sqrt(diffx * diffx + diffy * diffy)
                atandeg = np.degrees(np.arctan2(diffy, diffx))
                print(dis)
                if abs(nowrotate - atandeg) > 5:
                    state = 0
                    rob.drivedirect(0, 0)
                    break
                rob.drivedirect(100, 100)
                time.sleep(0.5)
                rob.drivedirect(0, 0)
                time.sleep(1)
        time.sleep(0.1)


t1 = threading.Thread(target=flaskThread)
t1.start()
# -----------------------------------------------------------

while True:
    # world plot
    plot = np.zeros((480, 640, 3), np.uint8)
    plot = cv2.circle(plot, (320, 240), 5, (0, 255, 0), 1)

    ret, frame = cap.read()

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict)

    if np.all(ids != None):
        display = aruco.drawDetectedMarkers(frame, corners)
        size_of_marker = 0.3  # side lenght of the marker in meter
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, size_of_marker, mtx, dist
        )

        id1 = np.array([[0, 0, 0]])
        id2 = np.array([[0, 0, 0]])
        tmp = {}

        tmp_real_world_pos = [0, 0, 0]  # x,y,count
        for i in range(len(tvecs)):
            length_of_axis = 0.3
            if ids[i] == 1:

                if prev_t is not None:
                    b = 0.025
                    for j in range(len(tvecs[i][0])):
                        tvecs[i][0][j] = b * prev_t[j] + (1 - b) * tvecs[i][0][j]
                    for j in range(len(rvecs[i][0])):
                        rvecs[i][0][j] = b * prev_r[j] + (1 - b) * rvecs[i][0][j]

                display = aruco.drawAxis(
                    display, mtx, dist, rvecs[i], tvecs[i], length_of_axis
                )

                rs, _ = cv2.Rodrigues(rvecs[i])
                if r is not None:
                    for a in range(r.shape[0]):
                        for b in range(r.shape[1]):
                            g = 0.95
                            rs[a][b] = rs[a][b] * (1 - g) + r[a][b] * g
                r = rs
                real_rotation = np.rad2deg(np.arctan2(rs[0][1], rs[1][1]))
                cam_pos, q = get_worldPos_from_aruco(tvecs[i], rs)
                # rsave.append(rvecs[i][0])
                # print(ids[i], worldPos)
                # tmp[ids[i][0]] = [tvecs[i].tolist(), rvecs[i].tolist()]
                # print(tmp)
                # if ids[i] == 0:
                #     print(ids[i], worldPos)
                # print(ids[i], tvecs[i], rvecs[i], cam_pos)
                if q:
                    plot = cv2.circle(
                        plot,
                        (
                            320 + int(cam_pos[0] * 40),
                            240 + int(cam_pos[1] * 40),
                        ),
                        5,
                        (0, 0, 255),
                        1,
                    )
                    rsave.append(rs)
                # real_world_pos = cam_pos
                tmp_real_world_pos[0] += cam_pos[0]
                tmp_real_world_pos[1] += cam_pos[1]
                tmp_real_world_pos[2] += 1
                # print(ids[i],real_world_pos)

                prev_t = tvecs[i][0]
                prev_r = rvecs[i][0]

            elif ids[i] == 0:
                if prev_t0 is not None:
                    b = 0.025
                    for j in range(len(tvecs[i][0])):
                        tvecs[i][0][j] = b * prev_t0[j] + (1 - b) * tvecs[i][0][j]
                    for j in range(len(rvecs[i][0])):
                        rvecs[i][0][j] = b * prev_r0[j] + (1 - b) * rvecs[i][0][j]

                display = aruco.drawAxis(
                    display, mtx, dist, rvecs[i], tvecs[i], length_of_axis
                )

                rs, _ = cv2.Rodrigues(rvecs[i])
                if r0 is not None:
                    for a in range(r0.shape[0]):
                        for b in range(r0.shape[1]):
                            g = 0.95
                            rs[a][b] = rs[a][b] * (1 - g) + r0[a][b] * g
                r0 = rs
                real_rotation = np.rad2deg(np.arctan2(rs[0][1], rs[1][1]))
                cam_pos, q = get_worldPos_from_aruco(tvecs[i], rs)
                # rsave.append(rvecs[i][0])
                # print(ids[i], worldPos)
                # tmp[ids[i][0]] = [tvecs[i].tolist(), rvecs[i].tolist()]
                # print(tmp)
                # if ids[i] == 0:
                #     print(ids[i], worldPos)
                cam_pos[0] += 1.63 + 0.46
                cam_pos[1] += -1.815 + 0.08
                # print(ids[i], tvecs[i], rvecs[i], cam_pos)
                if q:
                    plot = cv2.circle(
                        plot,
                        (
                            320 + int(cam_pos[0] * 40),
                            240 + int(cam_pos[1] * 40),
                        ),
                        5,
                        (255, 255, 255),
                        1,
                    )
                    rsave.append(rs)
                # real_world_pos = cam_pos
                tmp_real_world_pos[0] += cam_pos[0]
                tmp_real_world_pos[1] += cam_pos[1]
                tmp_real_world_pos[2] += 1
                # print(ids[i],real_world_pos)
                prev_t0 = tvecs[i][0]
                prev_r0 = rvecs[i][0]
            elif ids[i] == 2:
                if prev_t2 is not None:
                    b = 0.025
                    for j in range(len(tvecs[i][0])):
                        tvecs[i][0][j] = b * prev_t2[j] + (1 - b) * tvecs[i][0][j]
                    for j in range(len(rvecs[i][0])):
                        rvecs[i][0][j] = b * prev_r2[j] + (1 - b) * rvecs[i][0][j]

                display = aruco.drawAxis(
                    display, mtx, dist, rvecs[i], tvecs[i], length_of_axis
                )

                rs, _ = cv2.Rodrigues(rvecs[i])
                if r2 is not None:
                    for a in range(r2.shape[0]):
                        for b in range(r2.shape[1]):
                            g = 0.95
                            rs[a][b] = rs[a][b] * (1 - g) + r2[a][b] * g
                r2 = rs
                real_rotation = np.rad2deg(np.arctan2(rs[0][1], rs[1][1])) - 90
                real_rotation += (real_rotation < -180) * 360 - (
                    real_rotation > 180
                ) * 360
                cam_pos, q = get_worldPos_from_aruco(tvecs[i], rs)
                # rsave.append(rvecs[i][0])
                # print(ids[i], worldPos)
                # tmp[ids[i][0]] = [tvecs[i].tolist(), rvecs[i].tolist()]
                # print(tmp)
                # if ids[i] == 0:
                #     print(ids[i], worldPos)
                cam_pos[0], cam_pos[1] = -cam_pos[1], cam_pos[0]
                cam_pos[0] += -1.934 - 0.05
                cam_pos[1] += 1.887 - 0.14
                # print(ids[i], tvecs[i], rvecs[i], cam_pos)
                if q:
                    plot = cv2.circle(
                        plot,
                        (
                            320 + int(cam_pos[0] * 40),
                            240 + int(cam_pos[1] * 40),
                        ),
                        5,
                        (255, 255, 0),
                        1,
                    )
                    rsave.append(rs)
                # real_world_pos = cam_pos
                tmp_real_world_pos[0] += cam_pos[0]
                tmp_real_world_pos[1] += cam_pos[1]
                tmp_real_world_pos[2] += 1
                # print(ids[i],real_world_pos)
                prev_t2 = tvecs[i][0]
                prev_r2 = rvecs[i][0]
            elif ids[i] == 3:
                if prev_t3 is not None:
                    b = 0.025
                    for j in range(len(tvecs[i][0])):
                        tvecs[i][0][j] = b * prev_t3[j] + (1 - b) * tvecs[i][0][j]
                    for j in range(len(rvecs[i][0])):
                        rvecs[i][0][j] = b * prev_r3[j] + (1 - b) * rvecs[i][0][j]

                display = aruco.drawAxis(
                    display, mtx, dist, rvecs[i], tvecs[i], length_of_axis
                )

                rs, _ = cv2.Rodrigues(rvecs[i])
                if r3 is not None:
                    for a in range(r3.shape[0]):
                        for b in range(r3.shape[1]):
                            g = 0.95
                            rs[a][b] = rs[a][b] * (1 - g) + r3[a][b] * g
                r3 = rs
                real_rotation = np.rad2deg(np.arctan2(rs[0][1], rs[1][1])) - 90
                real_rotation += (real_rotation < -180) * 360 - (
                    real_rotation > 180
                ) * 360
                cam_pos, q = get_worldPos_from_aruco(tvecs[i], rs)
                # rsave.append(rvecs[i][0])
                # print(ids[i], worldPos)
                # tmp[ids[i][0]] = [tvecs[i].tolist(), rvecs[i].tolist()]
                # print(tmp)
                # if ids[i] == 0:
                #     print(ids[i], worldPos)
                cam_pos[0], cam_pos[1] = -cam_pos[1], cam_pos[0]
                cam_pos[0] += 1.916 - 0.07
                cam_pos[1] += 1.252 + 0.09
                # print(ids[i], tvecs[i], rvecs[i], cam_pos)
                if q:
                    plot = cv2.circle(
                        plot,
                        (
                            320 + int(cam_pos[0] * 40),
                            240 + int(cam_pos[1] * 40),
                        ),
                        5,
                        (255, 0, 255),
                        1,
                    )
                    rsave.append(rs)
                # real_world_pos = cam_pos
                tmp_real_world_pos[0] += cam_pos[0]
                tmp_real_world_pos[1] += cam_pos[1]
                tmp_real_world_pos[2] += 1

                prev_t3 = tvecs[i][0]
                prev_r3 = rvecs[i][0]
                # print(ids[i],real_world_pos)
        # print(real_world_pos)
        # lung for leaw na
        if tmp_real_world_pos[2] > 0:
            real_world_pos[0] = tmp_real_world_pos[0] / tmp_real_world_pos[2]
            real_world_pos[1] = tmp_real_world_pos[1] / tmp_real_world_pos[2]
        plot = cv2.circle(
            plot,
            (
                320 + int(real_world_pos[0] * 40),
                240 + int(real_world_pos[1] * 40),
            ),
            1,
            (255, 255, 255),
            -1,
        )

        cv2.imshow("Display", display)
        cv2.imshow("Plot", plot)
    else:
        display = frame
        cv2.imshow("Display", display)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
rsave = np.array(rsave)
t1.join()

import matplotlib.pyplot as plt

# plt.scatter(rsave[:, 0], rsave[:, 1], c="r")
# plt.scatter(rsave[:, 0], rsave[:, 2], c="b")
fig, ax = plt.subplots(2, 1)
ax[0].scatter(rsave[:, 0, 0], rsave[:, 0, 1], c="r", alpha=0.1, s=100)
ax[1].plot(range(len(rsave)), [r[2, 2] for r in rsave])
plt.show()
