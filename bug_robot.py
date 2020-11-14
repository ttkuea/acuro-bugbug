import threading
from robot.robot import Robot
from robot.state_machine import StateMachine

# Initialize 
robot = Robot()
is_running = True
state = 0 
state_desc = [
    '0 - set target',
    '1 - align target',
    '2 - move forward',
    '3 - follow the wall',
    '4 - finish'
    ]
pos = [0,0]
dest = None 
orient = 0 # Degree


# Main loop
while is_running:
    
    stateAction(state)

    stateTransition(state)









def stateAction(state):
    if state == 0:
        pass
    elif state == 1:
        pass
    elif state == 2:
        pass
    elif state == 3:
        pass
    elif state == 4:
        pass

def stateTransition(state):
    if state == 0:
        pass
    elif state == 1:
        pass
    elif state == 2:
        pass
    elif state == 3:
        pass
    elif state == 4:
        pass
        