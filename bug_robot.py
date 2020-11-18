import threading
import time

import cv2
import numpy as np

from robot.positioning import GlobalData
from robot.robot import Robot
from robot.state_machine import StateMachine

sta = GlobalData()

# while True:
#     print(sta.getPosition(), sta.getRotation())
#     cv2.waitKey(20)

# Initialize
robot = Robot()
is_running = True
state = 0
substate = 0  # substate for state 3 - follow wall
state_desc = [
    "0 - set target",
    "1 - align target",
    "2 - move forward",
    "3 - follow the wall",
    "4 - finish",
]
init_pos = None
pos = [0, 0, 0]  # x, y, z
dest = None
orient = 0  # Degree
substate_orient = 0
move_spd = 200
rot_spd = 100

treshold = 0.0001

# --------------------------------------------------
def stateAction(robot):
    global is_running
    global state, substate
    global init_pos, pos, dest, orient, substate_orient
    global move_spd, rot_spd

    if state == 0:  # Set target
        while True:
            tg = input(
                "press c for capture target position: "
            )  # c for capture, q for quit
            if tg == "q":  # quit
                is_running = False
                break
            elif tg == "c":  # capture target
                dest = [
                    sta.getPosition()[0],
                    sta.getPosition()[1],
                ]  # TODO: set dest position
                print("Set target position: %f,%f" % (dest[0], dest[1]))
            elif tg == "s":
                init_pos = [
                    sta.getPosition()[0],
                    sta.getPosition()[1],
                ]  # TODO: set init position
                print("Set initial position: %f,%f" % (init_pos[0], init_pos[1]))
                break

    elif state == 1:  # align target
        robot.drivedirect(rot_spd, -rot_spd)
        time.sleep(0.2)
        robot.drivedirect(0, 0)
        time.sleep(1)

    elif state == 2:  # move forward
        robot.drivedirect(move_spd, move_spd)

    elif state == 3:  # follow wall
        if substate == 0:  # rotate 45 degree
            robot.drivedirect(-rot_spd, rot_spd)
            time.sleep(0.5)
        elif substate == 1:
            robot.drivedirect(move_spd, int(move_spd * 0.4))
            time.sleep(1)
        elif substate == 2:
            robot.drivedirect(-int(rot_spd * 0.8), int(rot_spd * 0.8))
            time.sleep(0.5)

    elif state == 4:  # finish
        robot.drivedirect(0, 0)
        pass


# --------------------------------------------------
def stateTransition(robot):
    global is_running
    global state, substate
    global init_pos, pos, dest, orient, substate_orient
    global move_spd, rot_spd
    global treshold

    if state == 0:  # Set target -> align target
        state = 1

    elif state == 1:  # align target -> move forward
        align_data = is_allign(pos, dest, orient)
        atandeg = align_data[0]
        nowRotate = align_data[1]
        if abs(atandeg - nowRotate) < 10 or (atandeg > nowRotate and abs(atandeg - nowRotate + 360) < 10) or (atandeg < nowRotate and abs(nowRotate - atandeg + 360) < 10 ):
            state = 2


    elif state == 2:  # move forward -> follow wall, finish
        if robot.data.bumperL == 1 or robot.data.bumperR == 1:
            state = 3
            substate_orient = orient
        # TODO: xxx
        if (pos[0] - dest[0]) ** 2 + (pos[1] - dest[1]) ** 2 <= treshold ** 2:
            state = 4

    elif state == 3:  # follow wall -> align target
        if substate == 0:
            substate = 1
        elif substate == 1:
            substate = 2
        elif substate == 2:
            substate = 1
        # TODO: edit m_line
        if is_on_mline(init_pos, pos, dest):
            state = 1

    elif state == 4:  # finish
        pass


# --------------------------------------------------
def is_allign(pos, dest, orient):  # pos [x,y,r]
    nowrotate = 360 - ((orient + 630) % 360)
    diffx = dest[0] - pos[0]
    diffy = dest[1] - pos[1]
    atandeg = (np.degrees(np.arctan2(diffy, diffx)) + 360) % 360
    degreediff = (atandeg - nowrotate + 360) % 360
    print(degreediff, atandeg, nowrotate)
    return (
        atandeg,
        nowrotate,
        abs(nowrotate - atandeg) <= 5,
    )  # (nowrotate, degreediff, is_allign)


def is_on_mline(init_pos, pos, dest):
    s = np.sqrt((init_pos[0] - dest[0]) ** 2 + (init_pos[1] - dest[1]) ** 2)
    d1 = np.sqrt((init_pos[0] - pos[0]) ** 2 + (init_pos[1] - pos[1]) ** 2)
    d2 = np.sqrt((pos[0] - dest[0]) ** 2 + (pos[1] - dest[1]) ** 2)
    return d1 + d2 <= s


def real_orient(orient):
    real_o = (orient + 360) % 360
    print(real_o)
    return real_o


# --------------------------------------------------
# Main loop
cv2.namedWindow('plot')
while is_running:
    # TODO: get new pos
    pos = [sta.getPosition()[0], sta.getPosition()[1]]

    # TODO: get new orient
    orient = sta.getRotation()

    robot.readData()
    stateAction(robot)
    # robot.data.bumperL
    stateTransition(robot)

    cv2.imshow('plot', sta.getPlot())
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
