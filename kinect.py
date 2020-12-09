import cv2
import frame_convert2
import freenect

print('Press ESC in window to stop')


def get_depth():
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])


def get_video():
    return frame_convert2.video_cv(freenect.sync_get_video()[0])


while 1:
    cv2.imShow('Depth', get_depth())
    cv2.imShow('Video', get_video())
    if cv2.waitKey(10) == 27:
        break
