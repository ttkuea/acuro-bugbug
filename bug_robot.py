import threading
from robot.robot import Robot
from robot.state_machine import StateMachine
import cv2
import numpy as np
import time

# Initialize
robot = Robot()
is_running = True
state = 2 
substate = 0 # substate for state 3 - follow wall
state_desc = [
    '0 - set target',
    '1 - align target',
    '2 - move forward',
    '3 - follow the wall',
    '4 - finish'
    ]
init_pos = None
pos = [0,0,0] # x, y, z
dest = None 
orient = 0 # Degree
substate_orient = 0
move_spd = 200
rot_spd = 100

treshold = 0.0001

#--------------------------------------------------
def stateAction(robot):
    global is_running
    global state, substate
    global init_pos, pos, dest, orient, substate_orient
    global move_spd, rot_spd

    if state == 0: # Set target
        while dest is None and init_pos is None:
            tg = input('press c for capture target position: ') #c for capture, q for quit
            if tg == 'q': # quit
                is_running = False
                break
            elif tg == 'c': # capture target
                dest = [0,0] # TODO: set dest position
                print('Set target position: %f,%f' % (dest[0],dest[1]))
            elif tg == 's':
                init_pos = [0,0] #TODO: set init position
                print('Set initial position: %f,%f' % (init_pos[0],init_pos[1]))
        
    elif state == 1: # align target
        degreediff = is_allign(pos,dest,orient)[1]
        if degreediff > 180:
            robot.drivedirect(-rot_spd, rot_spd)
        else:
            robot.drivedirect(rot_spd, -rot_spd)

    elif state == 2: # move forward
        robot.drivedirect(move_spd, move_spd)

    elif state == 3: # follow wall
        print(substate)
        if substate == 0: # rotate 45 degree
            robot.drivedirect(-rot_spd, rot_spd)
            time.sleep(0.5)
        elif substate == 1:
            robot.drivedirect(move_spd, int(move_spd*0.4))
            time.sleep(1)
        elif substate == 2:
            robot.drivedirect(-int(rot_spd*0.8), int(rot_spd*0.8))
            time.sleep(0.5)
        
    elif state == 4: # finish
        robot.drivedirect(0,0)
        pass

#--------------------------------------------------
def stateTransition(robot):
    global is_running
    global state, substate
    global init_pos, pos, dest, orient, substate_orient
    global move_spd, rot_spd
    global treshold

    if state == 0: # Set target -> align target
        state = 1

    elif state == 1: # align target -> move forward
        isAlg = is_allign(pos,dest,orient)[2]
        if isAlg:
            state = 2
    
    elif state == 2: # move forward -> follow wall, finish
        if robot.data.bumperL == 1 or robot.data.bumperR == 1:
            state = 3
            substate_orient = orient
        # TODO: xxx
        # if (pos[0]-dest[0])**2 + (pos[1]-dest[1])**2 <= treshold**2:
        #     state = 4
        
    elif state == 3: # follow wall -> align target
        if substate == 0:
            substate = 1
        elif substate == 1:
            substate = 2
        elif substate == 2:
            substate = 1
        # TODO: edit m_line
        # if is_on_mline(init_pos, pos, dest):
        #     state = 1

    elif state == 4: # finish
        pass

#--------------------------------------------------
def is_allign(pos, dest, orient):# pos [x,y,r]
    nowrotate = 360 - ((orient + 630) % 360)
    diffx = dest[0] - pos[0]
    diffy = dest[1] - pos[1]
    atandeg = np.degrees(np.arctan2(diffy, diffx))
    degreediff = (atandeg - nowrotate + 360) % 360
    print(degreediff, atandeg, nowrotate)
    return (nowrotate, degreediff, abs(nowrotate - atandeg) <= 5) # (nowrotate, degreediff, is_allign)

def is_on_mline(init_pos, pos, dest):
    s = np.sqrt( (init_pos[0]-dest[0])**2 + (init_pos[1]-dest[1])**2  )
    d1 = np.sqrt( (init_pos[0]-pos[0])**2 + (init_pos[1]-pos[1])**2  )
    d2 = np.sqrt( (pos[0]-dest[0])**2 + (pos[1]-dest[1])**2  )
    return d1+d2 <= s

def real_orient(orient):
    real_o = (orient + 360) % 360
    print(real_o)
    return real_o

#--------------------------------------------------
# Main loop
while is_running:
    # TODO: get new pos

    # TODO: get new orient

    robot.readData()
    stateAction(robot)
    #robot.data.bumperL
    stateTransition(robot)