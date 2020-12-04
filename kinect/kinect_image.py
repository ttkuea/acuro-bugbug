from glob import glob

import cv2
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    # paths = glob('**/darrays/*-1606899156848835560.npy')
    paths = glob('**/darrays/*-1606899027735623744.npy')
    dpath = paths[0]
    print(dpath)
    idx = dpath[dpath.index('-') + 1: dpath.index('.npy')]
    ipath = glob('**/dimages/*' + idx + '.png')[0]

    depth = None
    with open(dpath, 'rb') as fp:
        depth = np.load(fp)
    
    for i in range(depth.shape[1]):
        X = depth[:, i].flatten()
        Y = [depth.shape[1] - y for y in range(depth.shape[0])]
        h = depth.shape[0]
        vfov = 53

        Y = [z * ((h//2 - j) * vfov / h) for j, z in enumerate(X)]

        plt.scatter(X, Y, c=X, s=1)

        cv2.imshow('Image', cv2.imread(ipath))

    plt.xlim(0, 256 * 5)

    plt.show()