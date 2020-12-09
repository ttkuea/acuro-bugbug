import freenect
import cv2
import numpy as np

def get_real_depth():
    array,_ = freenect.sync_get_depth()
    return array