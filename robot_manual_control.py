import struct
import time
import msvcrt
import serial


class RobotData:
    bumperR = 0
    bumperL = 0
    wheeldropR = 0
    wheeldropL = 0
    wheeldropCaster = 0
    wall = 0
    wallSignal = 0
    cliff0 = 0
    cliff1 = 0
    cliff2 = 0
    cliff3 = 0


class Robot:
    ser = None
    data = RobotData()

    def __init__(self):
        # Open new connection (NOTE, change port)
        self.ser = serial.Serial("COM3", baudrate=57600, timeout=0.5)

        # Actuator Commands
        # Send "Start" Opcode to start Open Interface, Roomba in Passive Mode
        self.ser.write(bytes([128]))
        self.ser.write(bytes([129, 10]))
        
        # Send "Safe Mode" Opcode to enable Roomba to respond to commands
        self.ser.write(bytes([131]))
        time.sleep(0.6)

    def drivedirect(self, left, right):
        m16l = struct.pack("h", left)
        m16r = struct.pack("h", right)
        self.ser.write(bytes([145, m16r[1], m16r[0], m16l[1], m16l[0]]))

    def readData(self):
        data = RobotData()
        self.ser.write(bytes([149, 7, 7, 8, 27, 28, 29, 30, 31]))
        self.ser.in_waiting

        # Bump and Wheeldrop
        tmp = self.ser.read(1)
        number = int.from_bytes(tmp, byteorder="big", signed=False)
        data.bumperR = number % 2
        data.bumperL = (number // 2) % 2
        data.wheeldropR = (number // 4) % 2
        data.wheeldropL = (number // 8) % 2
        data.wheeldropCaster = (number // 16) % 2

        # Wall
        tmp = self.ser.read(1)
        number = int.from_bytes(tmp, byteorder="big", signed=False)
        data.wall = number

        # Wall Signal
        tmp = self.ser.read(2)
        number = int.from_bytes(tmp, byteorder="big", signed=False)
        data.wallSignal = number

        # Cliffs
        tmp = self.ser.read(2)
        number = int.from_bytes(tmp, byteorder="big", signed=False)
        data.cliff0 = number
        tmp = self.ser.read(2)
        number = int.from_bytes(tmp, byteorder="big", signed=False)
        data.cliff1 = number
        tmp = self.ser.read(2)
        number = int.from_bytes(tmp, byteorder="big", signed=False)
        data.cliff2 = number
        tmp = self.ser.read(2)
        number = int.from_bytes(tmp, byteorder="big", signed=False)
        data.cliff3 = number
        self.data = data


rob = Robot()
rob.drivedirect(0, 0)
x = 0
y = 0
state = 0
spd = 200
c = b'p'

while True:
    rob.readData()

    # if msvcrt.kbhit():
    c = msvcrt.getch()  
    if c == b'w':
        x = y = spd
        #  rob.drivedirect(200, 200)
    elif c == b'a':
        x = -spd
        y = spd
        #  rob.drivedirect(-200, 200)
    elif c == b'd':
        x = spd
        y = -spd
        #  rob.drivedirect(200, -200)
    elif c == b's':
        x = y = -spd
        #  rob.drivedirect(-200, -200)
    elif c == b'q':
        x = y = 0
    elif c == b'p':
        x = y = 0
    
    rob.drivedirect(x,y)
    c = b'p'

    
    
    # roawaqb.drivedirect(0,0)
        
    
    # if state == 0:
    #     rob.drivedirect(200, 200)
    #     if rob.data.bumperL or rob.data.bumperR:
    #         state = 1
    # elif state == 1:
    #     rob.drivedirect(100, -100)
    #     time.sleep(2)
    #     state = 2
    # elif state == 2:
    #     rob.drivedirect(30, 200)
    #     if rob.data.bumperL or rob.data.bumperR:
    #         state = 3
    # elif state == 3:
    #     rob.drivedirect(100, -100)
    #     time.sleep(0.3)
    #     state = 2

    time.sleep(0.01)