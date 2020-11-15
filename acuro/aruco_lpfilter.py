import cv2
import numpy as np
import time

from threading import Thread

aruco_max_number = 4

class ArucoLPFilter:
    
    def __init__(self, delay=1.0/60.0):
        self.vector_function = None
        self.w = [0, 0]

        self.tvecs = [np.array([[0, 0, 0]])] * aruco_max_number
        self.rvecs = [np.array([[0, 0, 0]])] * aruco_max_number
        self.time = [-1] * aruco_max_number
        self.readys = [False] * aruco_max_number

        self.thread = None
        self.is_running = False
        self.last_update = -1
        self.delay = delay

    def bindInput(self, vector_function):
        self.vector_function = vector_function

    def setConstant(self, w):
        self.w = w

    def getVectors(self):
        return self.tvecs, self.rvecs, self.readys

    def update(self):
        # execute each loop
        now = time.time()
        dt = -1
        if self.last_update != -1:
            dt = (now - self.last_update)
        
        if dt == -1 or dt > self.delay:
            tvecs, rvecs, readys = self.vector_function()
            
            self.readys = [False] * aruco_max_number
            for i in range(4):
                ready = readys[i]
                tvec = tvecs[i]
                rvec = rvecs[i]

                if ready:

                    if self.time[i] == -1:
                        bt = 0
                        br = 0
                    else:
                        bt = np.exp(self.w[0] * (now - self.time[i]))
                        br = np.exp(self.w[1] * (now - self.time[i]))

                    self.tvecs[i] = bt * self.tvecs[i] + (1 - bt) * tvec
                    self.rvecs[i] = br * self.rvecs[i] + (1 - br) * rvec
                    self.time[i] = now
                    self.readys[i] = True

    def run(self):
        # thread target
        while self.is_running:
            self.update()
            time.sleep(0.001)

    def start(self):
        # start thread
        if not self.is_running:
            self.thread = Thread(target=self.run)
            self.is_running = True
            self.thread.start()

    def stop(self):
        # stop thread
        self.is_running = False