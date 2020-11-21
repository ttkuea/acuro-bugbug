import cv2
import numpy as np

from acuro.video import Video
from acuro.aruco_detector import ArucoDetector
from acuro.aruco_display import ArucoDisplay
from acuro.aruco_lpfilter import ArucoLPFilter
from acuro.aruco_particle_filter import ArucoParticleFilter
from acuro.aruco_estimator import ArucoEstimator


delay = 0.001

video = Video(input=1, fps=1000)
# video = Video(input='out/py_part1.mp4', fps=100)
video.start()

detector = ArucoDetector(delay=delay)
detector.bindInput(video.getImage)
detector.start()

lpFilter = ArucoLPFilter(delay=delay)
lpFilter.bindInput(detector.getVectors)
lpFilter.setConstant([-50, -50])
lpFilter.start()

particleFilter = ArucoParticleFilter(n_frame=5, delay=delay)
particleFilter.bindInput(lpFilter.getVectors)
particleFilter.start()

display = ArucoDisplay(detector.mtx, detector.dist, delay=delay)
display.bindIput(detector.getImage, lpFilter.getVectors)
display.start()

estimator = ArucoEstimator(delay=delay)
estimator.bindInput(lpFilter.getVectors)
estimator.setOffsets(
    [
        np.array([1.63 + 0.46, -1.815 + 0.08]),
        np.array([0.0, 0.0]),
        np.array([-1.934 - 0.05, 1.887 - 0.14]),
        np.array([1.916 - 0.07, 1.252 + 0.09]),
    ], 
    [ 0.0, 0.0, -90.0,-90.0]
)
estimator.start()

colors = [
    (255, 255, 255),
    (0, 0, 255),
    (255, 255, 0),
    (255, 0, 255),
]

window_name1 = 'Video'
window_name2 = 'Estimation'
window_name3 = 'Samples 01'
window_name4 = 'Samples 12'
cv2.namedWindow(window_name1)
cv2.namedWindow(window_name2)
cv2.namedWindow(window_name3)

try:
    while True:
        image = display.getImage()
        plot = np.zeros((480, 640, 3), np.uint8)
        plot = cv2.circle(plot, (320, 240), 5, (0, 255, 0), 1)
        plot2 = np.zeros((480, 480, 3), np.uint8)
        plot3 = np.zeros((480, 480, 3), np.uint8)

        # ts, rs, rds = detector.getVectors()
        # for r, rd in zip(rs, rds):
        #     if rd:
        #         print(r)
        #         break
    
        pos, rot, readys = estimator.getEstimation()
        for i in range(len(readys)):
            if readys[i]:
                plot = cv2.circle(
                    plot,
                    (
                        320 + int(pos[i][0] * 40),
                        240 + int(pos[i][1] * 40),
                    ),
                    5,
                    colors[i],
                    1,
                )

        samples, scores = particleFilter.getSamples()
        for i in [1, 3]:
            for j in range(samples.shape[1]):
                plot2 = cv2.circle(
                    plot2,
                    (
                        240 + int(samples[i][j][0] * 240 / np.pi),
                        240 + int(samples[i][j][1] * 240 / np.pi),
                    ),
                    int(scores[i][j] * 1000),
                    colors[i],
                    -1,
                    )
                plot3 = cv2.circle(
                    plot3,
                    (
                        240 + int(samples[i][j][1] * 240 / np.pi),
                        240 + int(samples[i][j][2] * 240 / np.pi),
                    ),
                    int(scores[i][j] * 1000),
                    colors[i],
                    -1,
                    )


        if image is not None:
            # print(video.last_update, acuro.last_update)
            cv2.imshow(window_name1, image)

        cv2.imshow(window_name2, plot)
        cv2.imshow(window_name3, np.concatenate((plot2, plot3), axis=1))

        if cv2.waitKey(16) & 0xFF == ord('q'):
            break
except Exception as e:
    print(e)

video.stop()
detector.stop()
display.stop()
lpFilter.stop()
particleFilter.stop()
estimator.stop()
cv2.destroyAllWindows()