import cv2
import numpy as np
import time

from threading import Thread

aruco_max_number = 4

class ArucoEstimator:
    
    def __init__(self, delay=1.0/60.0):
        self.vector_function = None
        self.positionOffsets = [np.array([0, 0])] * aruco_max_number
        self.rotationOffsets = [0.0] * aruco_max_number

        self.position = [np.array([0, 0])] * aruco_max_number
        self.rotation = [0.0] * aruco_max_number
        self.readys = [False] * aruco_max_number

        self.thread = None
        self.is_running = False
        self.last_update = -1
        self.delay = delay

    def bindInput(self, vector_function):
        self.vector_function = vector_function

    def setOffsets(self, pos_offsets, rot_offsets):
        if len(pos_offsets) != aruco_max_number:
            raise Exception('Need ' + str(aruco_max_number) + ' 2D Position Offsets!')

        if len(rot_offsets) != aruco_max_number:
            raise Exception('Need ' + str(aruco_max_number) + ' Rotation Offsets!')

        self.positionOffsets = pos_offsets
        self.rotationOffsets = rot_offsets

    def getEstimation(self):
        return self.position, self.rotation, self.readys

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
                    rodrigues, _ = cv2.Rodrigues(rvec)

                    # calculate position
                    extristics = np.matrix(
                        [
                            [rodrigues[0][0], rodrigues[0][1], rodrigues[0][2], tvec[0][0]],
                            [rodrigues[1][0], rodrigues[1][1], rodrigues[1][2], tvec[0][1]],
                            [rodrigues[2][0], rodrigues[2][1], rodrigues[2][2], tvec[0][2]],
                            [0.0, 0.0, 0.0, 1.0],
                        ]
                    )
                    extristics_I = extristics.I  # inverse matrix
                    worldPos = [
                        extristics_I[0, 3],
                        extristics_I[1, 3],
                        extristics_I[2, 3],
                    ]
                    position = np.array(worldPos[: 2]) + self.positionOffsets[i]

                    # calculate rotation
                    rotation = np.rad2deg(np.arctan2(rodrigues[0][1], 
                                                          rodrigues[1][1]))
                    rotation += self.rotationOffsets[i]
                    rotation += (rotation < -180) * 360 - (rotation > 180) * 360

                    self.readys[i] = True
                    self.position[i] = position
                    self.rotation[i] = rotation

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