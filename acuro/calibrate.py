from pprint import pprint

import cv2
import cv2.aruco as aruco
import numpy as np

cap = cv2.VideoCapture(2)
cap.set(3, 640)
cap.set(4, 480)

count = 0
while True:
    count += 1
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=arucoParameters
    )
    if count % 10 == 0 and ids is not None and len(ids) == 35:
        print("suchart capture", count)
        print(len(ids))
        cv2.imwrite("images/%d.jpg" % count, frame)
    display = aruco.drawDetectedMarkers(frame, corners)
    cv2.imshow("Display", display)
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()
