#import the necessary modules
import freenect
import cv2
import numpy as np
 
#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array

def get_real_depth():
    array,_ = freenect.sync_get_depth()
    return array

def get_real_dist(pixel):
    #THIS IS FUCKING MAGIC WE DON'T KNOW WHAT IT IS
    return 0.1236 * np.tan(pixel / 2842.5 + 1.1863)

 
if __name__ == "__main__":
    while 1:
        #get a frame from RGB camera
        # frame = get_video()
        #get a frame from depth sensor
        depth = get_depth()
        #display RGB image
        # cv2.imshow('RGB image',frame)
        #display depth image
        depth = cv2.circle(depth, (320,240), 3, (0,255,0), -1)

        cv2.imshow('Depth image',depth)
 
        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
        elif k == ord('a'):
            array,_ = freenect.sync_get_depth()
            # array, = freenect.get_depth()
            rawDisparity = array[240][320]
            print("YAYY", get_real_dist(rawDisparity))
            # print(array.shape)
            # print(array[0])
    cv2.destroyAllWindows()