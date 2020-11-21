import struct
import time

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
        self.ser = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=0.5)

        # Actuator Commands
        # Send "Start" Opcode to start Open Interface, Roomba in Passive Mode
        self.ser.write(bytes([128]))
        self.ser.write(bytes([129, 10]))
        # Send "Safe Mode" Opcode to enable Roomba to respond to commands
        self.ser.write(bytes([132]))
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
