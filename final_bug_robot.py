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
robot = Robot("/dev/ttyUSB0")
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
curr_pos = [0, 0]
dest = None
orient = 0  # Degree
substate_orient = 0
move_spd = 200
rot_spd = 50

mline_counter = 0

atandeg = 0

treshold = 21
robot.drivedirect(0, 0)

timer = 0


# --------------------------------------------------
def stateAction(robot):
    global is_running
    global state, substate
    global init_pos, pos, dest, orient, substate_orient, curr_pos
    global move_spd, rot_spd

    if state == 0:  # Set target
        """tg = input(
            "press c for capture target position: "
        )  # c for capture, q for quit
        """
        tg = cv2.waitKey(10) & 0xFF
        if tg == ord("q"):  # quit
            is_running = False
        elif tg == ord("c"):  # capture target
            dest = [
                sta.getPosition()[0],
                sta.getPosition()[1],
            ]  # TODO: set dest position
            dest = [-0.837853, -0.597194]  # TODO LOB DUAY SUS
            sta.setTarget(dest)
            print("Set target position: %f,%f" % (dest[0], dest[1]))
        elif tg == ord("s"):
            init_pos = [
                sta.getPosition()[0],
                sta.getPosition()[1],
            ]  # TODO: set init position
            sta.setStart(init_pos)
            state = 1
            print("Set initial position: %f,%f" % (init_pos[0], init_pos[1]))
        elif tg == ord("l"):
            check_occ()

    if state == 1:  # align target
        robot.drivedirect(rot_spd, -rot_spd)

    elif state == 2:  # move forward
        robot.drivedirect(move_spd, move_spd)

    elif state == 3:  # follow wall
        if substate == 0:  # rotate 45 degree
            robot.drivedirect(-100, 100)
        elif substate == 1:
            robot.drivedirect(100, 100)  # int(100 * 0.3))
        elif substate == 2:
            robot.drivedirect(int(100 * 0.8), -int(100 * 0.8))

    elif state == 4:  # finish
        robot.drivedirect(0, 0)
        pass

    elif state == 5:
        robot.drivedirect(rot_spd, -rot_spd)


# --------------------------------------------------
def stateTransition(robot):
    global is_running
    global state, substate
    global init_pos, pos, dest, orient, substate_orient, curr_pos
    global move_spd, rot_spd
    global treshold
    global atandeg
    global mline_counter
    global timer

    if state == 1:  # align target -> move forward
        diffx = dest[0] - init_pos[0]
        diffy = dest[1] - init_pos[1]
        print("Diff", diffx, diffy, np.degrees(np.arctan2(-diffx, -diffy)))
        atandeg = (np.degrees(np.arctan2(-diffx, -diffy)) + 360) % 360
        orient = sta.getRotation()
        # nowRotate = 360 - ((orient + 360) % 360)
        nowRotate = (orient + 377) % 360

        print("loop", nowRotate, atandeg)
        while not (
            abs(atandeg - nowRotate) < 5
            or (atandeg > nowRotate and abs(atandeg - nowRotate + 360) < 5)
            or (atandeg < nowRotate and abs(nowRotate - atandeg + 360) < 5)
        ):
            orient = sta.getRotation()
            cv2.imshow("Plot", sta.getPlot())
            cv2.imshow("Display", sta.getDisplay())
            # nowRotate = 360 - ((orient + 360) % 360)
            nowRotate = (orient + 377) % 360
            print(nowRotate, atandeg)
            cv2.waitKey(10)
        state = 2

    elif state == 2:  # move forward -> follow wall, finish
        if robot.data.bumperL == 1 or robot.data.bumperR == 1 or check_occ():
            # if robot.data.wallSignal > 25:
            state = 3
            substate_orient = orient
            timer = time.time()
        # TODO: xxx

        # print(pos,dest)
        # print((pos[0]*100 - dest[0]*100) ** 2 + (pos[1] - dest[1]) ** 2, treshold ** 2)
        if (pos[0] * 100 - dest[0] * 100) ** 2 + (
            pos[1] * 100 - dest[1] * 100
        ) ** 2 <= treshold ** 2:
            print("2 at target")
            state = 4

        # if check_occ():
        #     state = 3
        #     substate_orient = orient

    elif state == 3:  # follow wall -> align target
        if (pos[0] * 100 - dest[0] * 100) ** 2 + (
            pos[1] * 100 - dest[1] * 100
        ) ** 2 <= treshold ** 2:
            print("3 at target")
            state = 4
        if is_on_mline(init_pos, pos, dest) and mline_counter >= 50:
            # print("CHECK MLINE")
            state = 5
            mline_counter = 0
            curr_pos = list(pos)
        print(mline_counter)

        if substate == 0:
            if time.time() - timer > 2:
                substate = 1
                timer = time.time()
        elif substate == 1:
            if robot.data.bumperL == 1 or robot.data.bumperR == 1:
                mline_counter += 1
                substate = 0
                timer = time.time()
            # if robot.data.wallSignal > 25:
            if time.time() - timer > 3:
                if check_occ2():
                    mline_counter += 1
                    substate = 2
                    timer = time.time()
        elif substate == 2:
            if time.time() - timer > 2.1:
                substate = 1
                timer = time.time()
        # TODO: edit m_line

    elif state == 4:  # finish
        pass

    elif state == 5:  # rotate after mline
        diffx = dest[0] - init_pos[0]
        diffy = dest[1] - init_pos[1]
        print("Diff", diffx, diffy, np.degrees(np.arctan2(-diffx, -diffy)))
        atandeg = (np.degrees(np.arctan2(-diffx, -diffy)) + 360) % 360
        orient = sta.getRotation()
        # nowRotate = 360 - ((orient + 360) % 360)
        nowRotate = (orient + 377) % 360
        while not (
            abs(atandeg - nowRotate) < 5
            or (atandeg > nowRotate and abs(atandeg - nowRotate + 360) < 5)
            or (atandeg < nowRotate and abs(nowRotate - atandeg + 360) < 5)
        ):
            cv2.imshow("Plot", sta.getPlot())
            cv2.imshow("Display", sta.getDisplay())
            orient = sta.getRotation()
            # nowRotate = 360 - ((orient + 360) % 360)
            nowRotate = (orient + 377) % 360
            print(nowRotate, atandeg)
            cv2.waitKey(10)
        state = 2


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
    # print(d1+d2,s)
    return d1 + d2 <= s + 0.01


def real_orient(orient):
    real_o = (orient + 360) % 360
    print(real_o)
    return real_o


def check_occ():
    global pos
    global orient
    # Kinect
    resolution = 0.1525
    # occupancy_grid.min_treshold = -50
    # occupancy_grid.max_treshold = 50
    min_coordinate = resolution * 20
    treshold = 200

    occ_grid = sta.getOccupancy()
    # print(occ_grid)

    robot_x = (pos[0] + min_coordinate) // resolution
    robot_y = (pos[1] + min_coordinate) // resolution

    robot_orient = (orient + 377) % 360

    # ccw = ++++++++ naja
    # print(robot_x, robot_y)
    if 0 < robot_x < 39 and 0 < robot_y < 39:
        # for i in range(-1,2):
        #     for j in range(-1,2):
        #         if i != 1 and j != 1:
        #             if occ_grid[int(robot_y - j), int(robot_x - i)] > treshold:
        #                 print(True)
        #                 return True

        if 0 <= robot_orient < 45 or 315 <= robot_orient < 360:
            print("Front")
            for i in range(-1, 2):
                for j in range(-3, 0):  # -1,1
                    if i != 1 and j != 1:
                        print(
                            robot_x,
                            robot_y,
                            int(robot_x + j),
                            int(robot_y + i),
                            occ_grid[int(robot_y + j), int(robot_x + i)],
                        )
                        if occ_grid[int(robot_y + j), int(robot_x + i)] > treshold:
                            print(True)
                            return True

        elif 45 <= robot_orient < 135:
            print("Left")
            for i in range(-3, 0):  # -1,1
                for j in range(-1, 2):
                    if i != 1 and j != 1:
                        print(
                            robot_x,
                            robot_y,
                            int(robot_x + j),
                            int(robot_y + i),
                            occ_grid[int(robot_y + j), int(robot_x + i)],
                        )
                        if occ_grid[int(robot_y + j), int(robot_x + i)] > treshold:
                            print(True)
                            return True

        elif 135 <= robot_orient < 225:
            print("Down")
            for i in range(-1, 2):
                for j in range(1, 4):  # 0,2
                    if i != 1 and j != 1:
                        print(
                            robot_x,
                            robot_y,
                            int(robot_x + j),
                            int(robot_y + i),
                            occ_grid[int(robot_y + j), int(robot_x + i)],
                        )
                        if occ_grid[int(robot_y + j), int(robot_x + i)] > treshold:
                            print(True)
                            return True

        elif 225 <= robot_orient < 315:
            print("Right")
            for i in range(1, 4):  # 0,2
                for j in range(-1, 2):
                    if i != 1 and j != 1:
                        print(
                            robot_x,
                            robot_y,
                            int(robot_x + j),
                            int(robot_y + i),
                            occ_grid[int(robot_y + j), int(robot_x + i)],
                        )
                        if occ_grid[int(robot_y + j), int(robot_x + i)] > treshold:
                            print(True)
                            return True

    # print(False)
    return False


def check_occ2():
    global pos
    global orient
    # Kinect
    resolution = 0.1525
    # occupancy_grid.min_treshold = -50
    # occupancy_grid.max_treshold = 50
    min_coordinate = resolution * 20
    treshold = 200

    occ_grid = sta.getOccupancy()
    # print(occ_grid)

    robot_x = (pos[0] + min_coordinate) // resolution
    robot_y = (pos[1] + min_coordinate) // resolution

    robot_orient = (orient + 377) % 360

    if 0 < robot_x < 39 and 0 < robot_y < 39:
        if 0 <= robot_orient < 45 or 315 <= robot_orient < 360:
            print("Front 2")
            # print(occ_grid[int(robot_y - 1), int(robot_x + 1)] < treshold)
            return np.all(
                occ_grid[
                    int(robot_y - 3) : int(robot_y + 1) + 1,
                    int(robot_x + 1) : int(robot_x + 3) + 1,
                ]
                < treshold
            )

        elif 45 <= robot_orient < 135:
            print("Left 2")
            return np.all(
                occ_grid[
                    int(robot_y - 3) : int(robot_y - 1) + 1,
                    int(robot_x - 3) : int(robot_x + 1) + 1,
                ]
                < treshold
            )

        elif 135 <= robot_orient < 225:
            print("Down 2")
            return np.all(
                occ_grid[
                    int(robot_y) : int(robot_y + 3) + 1,
                    int(robot_x - 3) : int(robot_x - 1) + 1,
                ]
                < treshold
            )

        elif 225 <= robot_orient < 315:
            print("Right 2")
            return np.all(
                occ_grid[
                    int(robot_y + 1) : int(robot_y + 3) + 1,
                    int(robot_x) : int(robot_x + 3) + 1,
                ]
                < treshold
            )


# --------------------------------------------------
# Main loop
# --------------------------------------------------
cv2.namedWindow("Plot", cv2.WINDOW_NORMAL)
cv2.namedWindow("Display", cv2.WINDOW_NORMAL)
cv2.namedWindow("Occupancy", cv2.WINDOW_NORMAL)
count = 0
while is_running:
    # TODO: get new pos
    pos = [sta.getPosition()[0], sta.getPosition()[1]]

    # TODO: get new orient
    orient = sta.getRotation()

    robot.readData()
    stateAction(robot)
    # robot.data.bumperL
    stateTransition(robot)

    cv2.imshow("Plot", sta.getPlot())
    cv2.imshow("Display", sta.getDisplay())
    occ = cv2.cvtColor(sta.getOccupancy(), cv2.COLOR_GRAY2BGR)
    occ[
        int((pos[1] + 0.1525 * 20) // 0.1525), int((pos[0] + 0.1525 * 20) // 0.1525)
    ] = [0, 255, 0]
    cv2.imshow("Occupancy", occ)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    count += 1
    if count % 50 == 0:
        print(pos)
robot.drivedirect(0, 0)
