import cv2
import cv2.aruco as aruco
import numpy as np
import time
import yaml

from glob import glob
from threading import Thread

aruco_max_number = 4

class ArucoDetector:
    
    def __init__(self, delay=1.0/60.0):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        # cap.set(5, 60)

        calibration_paths = glob('**/*calibration*.yaml')
        calibration_file = None

        if len(calibration_paths) > 1:
            while calibration_file is None:
                print('please specify calibration file:')
                for i, path in enumerate(calibration_paths):
                    print('[' + str(i) + ']', path)
                idx = int(input('input: ').strip())
                if idx < len(calibration_paths):
                    calibration_file = calibration_paths[i]
                else:
                    print('Invalid index!')
        else:
            calibration_file = calibration_paths[0]

        print('use', calibration_file)

        with open(calibration_file, 'r') as fp:
            loadeddict = yaml.load(fp, Loader=yaml.FullLoader)
            self.mtx = loadeddict.get("camera_matrix")
            self.dist = loadeddict.get("dist_coeff")
            self.mtx = np.array(self.mtx)
            self.dist = np.array(self.dist)

        self.input_function = None
        self.image= None

        self.tvecs = [np.array([[0, 0, 0]])] * aruco_max_number
        self.rvecs = [np.array([[0, 0, 0]])] * aruco_max_number
        self.readys = [False] * aruco_max_number

        self.size_of_marker = 0.3

        self.thread = None
        self.is_running = False
        self.last_update = -1
        self.delay = delay

    def bindInput(self, input_function):
        self.input_function = input_function

    def getImage(self):
        return self.image

    def getVectors(self):
        return self.tvecs, self.rvecs, self.readys

    def run(self):
        while self.is_running:
            now = time.time()
            dt = -1
            if self.last_update != -1:
                dt = (now - self.last_update)

            # print(dt)
            if dt == -1 or dt > self.delay:
                frame = self.input_function()

                if frame is not None:
                    self.readys = [False] * aruco_max_number

                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    # gray = cv2.GaussianBlur(gray, (3, 3), 5)
                    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
                    arucoParameters = aruco.DetectorParameters_create()
                    corners, ids, rejectedImgPoints = aruco.detectMarkers(
                        gray, aruco_dict, parameters=arucoParameters
                    )
                    if np.all(ids != None):
                        frame = aruco.drawDetectedMarkers(frame, corners)

                        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                            corners, self.size_of_marker, self.mtx, self.dist
                        )

                        for i, idx in enumerate(ids.flatten()):
                            self.readys[idx] = True
                            self.tvecs[idx] = tvecs[i]
                            self.rvecs[idx] = rvecs[i]
                    
                        self.last_update = now
                self.image = frame
                        
            time.sleep(0.001)

    def start(self):
        if not self.is_running:
            self.thread = Thread(target=self.run)
            self.is_running = True
            self.thread.start()

    def stop(self):
        self.is_running = False