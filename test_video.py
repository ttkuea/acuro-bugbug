import cv2
import numpy as np

from acuro.video import Video
from acuro.aruco_detector import ArucoDetector
from acuro.aruco_display import ArucoDisplay
from acuro.aruco_estimator import ArucoEstimator


video = Video('outpy_part1.mp4', fps=1000)
video.start()

aruco1 = ArucoDetector(delay=0.001)
aruco1.bindInput(video.getImage)
aruco1.start()

aruco2 = ArucoDisplay(aruco1.mtx, aruco1.dist, delay=0.001)
aruco2.bindIput(aruco1.getImage, aruco1.getVectors)
aruco2.start()

aruco3 = ArucoEstimator(delay=0.001)
aruco3.bindInput(aruco1.getVectors)
aruco3.setOffsets(
    [
        np.array([1.63 + 0.46, -1.815 + 0.08]),
        np.array([0.0, 0.0]),
        np.array([-1.934 - 0.05, 1.887 - 0.14]),
        np.array([1.916 - 0.07, 1.252 + 0.09]),
    ], 
    [ 0.0, 0.0, -90.0,-90.0]
)
aruco3.start()

window_name1 = 'Video'
window_name2 = 'Estimation'
cv2.namedWindow(window_name1)

while True:
    image = aruco2.getImage()
    plot = np.zeros((480, 640, 3), np.uint8)
    plot = cv2.circle(plot, (320, 240), 5, (0, 255, 0), 1)

    pos, rot, readys = aruco3.getEstimation()
    for p, _, rd in zip(pos, rot, readys):
        if rd:
            plot = cv2.circle(
                plot,
                (
                    320 + int(p[0] * 40),
                    240 + int(p[1] * 40),
                ),
                5,
                (0, 0, 255),
                1,
            )

    if image is not None:
        # print(video.last_update, acuro.last_update)
        cv2.imshow(window_name1, image)

    cv2.imshow(window_name2, plot)

    if cv2.waitKey(16) & 0xFF == ord('q'):
        break

video.stop()
aruco1.stop()
aruco2.stop()
aruco3.stop()
cv2.destroyAllWindows()