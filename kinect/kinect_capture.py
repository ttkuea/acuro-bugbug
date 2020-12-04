from pathlib import Path

import cv2
import freenect
import numpy as np
import os
import time


window_name = 'Kinect Depth'


root = Path(__file__).parent.absolute()

save_depth_dir = root.joinpath('../data/darrays/')
save_image_dir = root.joinpath('../data/dimages/')

if __name__ == "__main__":

    freenect.init()
    cv2.namedWindow(window_name)

    while True:

        depth, _ = freenect.sync_get_depth()
        depth = np.resize(depth, (480, 640, 1))
        depth_0 = np.where(np.logical_and(depth > 256 * 2, depth < 256 * 3), depth - 256 * 2, 0) + np.where(depth > 256 * 3, 255, 0)
        depth_1 = np.where(np.logical_and(depth > 256 * 3, depth < 256 * 4), depth - 256 * 3, 0) + np.where(depth > 256 * 4, 255, 0)
        depth_2 = np.where(np.logical_and(depth > 256 * 4, depth < 256 * 5), depth - 256 * 4, 0) + np.where(depth > 256 * 5, 255, 0)
        depth_rgb = np.concatenate([depth_2, depth_1, depth_0], axis=-1)
        # print(depth_rgb.shape)

        cv2.imshow(window_name, depth_rgb.astype(np.uint8))

        wkey = cv2.waitKey(1)
        if wkey & 0xFF == ord("c"):
            time_str = str(time.time_ns())
            save_depth_name = str(save_depth_dir.joinpath('darray-' + time_str + '.npy'))
            save_image_name = str(save_image_dir.joinpath('dimage-' + time_str + '.png'))

            with open(save_depth_name, 'wb') as fp:
                np.save(fp, depth)
            print('saved depth to ' + save_depth_name)

            cv2.imwrite(save_image_name, depth_rgb)
            print('saved image to ' + save_image_name)

        if wkey & 0xFF == ord("q"):
            break