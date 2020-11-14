import cv2
import cv2.aruco as aruco
import numpy as np
import time
import yaml

from glob import glob
from threading import Thread


class ArucoDisplay:

    def __init__(self, mtx, dist, delay=1.0/60.0):
        self.mtx = mtx
        self.dist = dist
        self.image_function = None
        self.vector_function = None

        self.image = None
        self.length_of_axis = 1

        self.thread = None
        self.is_running = False
        self.last_update = -1
        self.delay = delay

    def bindIput(self, image_function, vector_function):
        self.image_function = image_function
        self.vector_function = vector_function

    def getImage(self):
        return self.image

    def run(self):
        while self.is_running:
            now = time.time()
            dt = -1
            if self.last_update != -1:
                dt = (now - self.last_update)

            # print(dt)
            if dt == -1 or dt > self.delay:
                frame = self.image_function()

                if frame is not None:
                    tvecs, rvecs, readys = self.vector_function()
                    
                    # print(readys)
                    for i in range(4):
                        ready = readys[i]
                        tvec = tvecs[i]
                        rvec = rvecs[i]

                        if ready == 1:
                            frame = aruco.drawAxis(
                                frame, self.mtx, self.dist, rvec, tvec, self.length_of_axis
                            )

                    self.image = frame
                    self.last_update = now
                        
            time.sleep(0.001)

    def start(self):
        if not self.is_running:
            self.thread = Thread(target=self.run)
            self.is_running = True
            self.thread.start()

    def stop(self):
        self.is_running = False