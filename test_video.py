import cv2

from acuro.video import Video
from acuro.aruco_detector import ArucoDetector
from acuro.aruco_display import ArucoDisplay


video = Video('outpy_part1.mp4', fps=1000)
video.start()

aruco1 = ArucoDetector(delay=0.001)
aruco1.bindInput(video.getImage)
aruco1.start()

aruco2 = ArucoDisplay(aruco1.mtx, aruco1.dist, delay=0.001)
aruco2.bindIput(aruco1.getImage, aruco1.getVectors)
aruco2.start()

window_name = 'Video'
cv2.namedWindow(window_name)

while True:
    image = aruco2.getImage()
    if image is not None:
        # print(video.last_update, acuro.last_update)
        cv2.imshow(window_name, image)

    if cv2.waitKey(16) & 0xFF == ord('q'):
        break

video.stop()
aruco1.stop()
aruco2.stop()
cv2.destroyAllWindows()