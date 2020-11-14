import cv2
import numpy as np
import time

from scipy.special import i0
from threading import Thread

aruco_max_number = 4

class ArucoParticleFilter:
    
    def __init__(self, n_random=50, n_frame=10, delay=1.0/60.0):
        self.vector_function = None

        self.tvecs = [np.array([[0, 0, 0]])] * aruco_max_number
        self.rvecs = [np.array([[0, 0, 0]])] * aruco_max_number
        self.readys = [False] * aruco_max_number

        self.n_random = n_random
        self.n_frame = n_frame
        self.rSamples = np.random.uniform(- np.pi, np.pi, (aruco_max_number, n_random, 3))
        self.scores = np.zeros((aruco_max_number, n_random))
        # print(self.rSamples.shape)

        self.thread = None
        self.is_running = False
        self.last_update = -1
        self.delay = delay

    def bindInput(self, vector_function):
        self.vector_function = vector_function

    def getVectors(self):
        return self.tvecs, self.rvecs, self.readys

    def getSamples(self):
        return self.rSamples, self.scores

    def update(self):
        # execute each loop
        now = time.time()
        dt = -1
        if self.last_update != -1:
            dt = (now - self.last_update)
        
        if dt == -1 or dt > self.delay:
            self.readys = [False] * aruco_max_number
            tvecs_stack = [[], [], [], []]
            rvecs_stack = [[], [], [], []]

            # get measurement
            for _ in range(self.n_frame):
                tvecs, rvecs, readys = self.vector_function()
                # print(readys)
                
                for i in range(aruco_max_number):
                    ready = readys[i]
                    tvec = tvecs[i]
                    rvec = rvecs[i]

                    if ready:
                        # print(i)
                        tvecs_stack[i].append(tvec)
                        rvecs_stack[i].append(rvec)

                time.sleep(self.delay)

            # print([len(rvecs_stack[i]) for i in range(aruco_max_number)])

            # predict
            m = self.n_random * 2
            new_samples = []
            for i in range(aruco_max_number):
                new_sample = []
                for j in range(self.rSamples.shape[1]):
                    x, y, z = self.rSamples[i][j]
                    X = np.random.vonmises(x, 0.2, m)
                    Y = np.random.vonmises(y, 0.2, m)
                    Z = np.random.vonmises(z, 0.2, m)
                    new_sample += list(zip(X, Y, Z))
                new_samples.append(new_sample)
            new_samples = np.array(new_samples)
            # print(new_samples.shape)

            # update scores
            e = 1.5
            def score(x, mu):
                return np.exp(e * np.cos(x - mu))/(2 * np.pi * i0(e))

            scores = np.zeros((aruco_max_number, new_samples.shape[1]))
            for i in range(aruco_max_number):
                # print(rvecs_stack[i])
                for j in range(new_samples.shape[1]):
                    for k in range(len(rvecs_stack[i])):
                        # print(rvecs_stack[i][k][0])
                        x, y, z = new_samples[i][j]
                        x0, y0, z0 = rvecs_stack[i][k][0]
                        scores[i][j] += score(x, x0) * score(y, y0) * score(z, z0)
                        # print(scores[i][j])

                score_sum = sum(scores[i])
                # print(score_sum)
                if score_sum != 0:
                    for j in range(new_samples.shape[1]):
                        scores[i][j] /= score_sum
                elif len(scores[i]) > 0:
                    for j in range(new_samples.shape[1]):
                        scores[i][j] = 1 / len(scores[i])

            # resampling
            resamples_idx = [np.random.choice(range(new_samples.shape[1]), size=self.n_random, p=scores[i]) for i in range(aruco_max_number)]
            resamples = np.array([[new_samples[i][idx] for idx in resamples_idx[i]] for i in range(aruco_max_number)])
            self.rSamples = resamples
            self.scores = np.array([[scores[i][idx] for idx in resamples_idx[i]] for i in range(aruco_max_number)])


            # average
            for i in range(aruco_max_number):
                n = (len(tvecs_stack[i]) == len(rvecs_stack[i])) * len(tvecs_stack[i])

                if n > 0:
                    self.tvecs[i] = sum(tvecs_stack[i]) / n
                    self.rvecs[i] = sum(rvecs_stack[i]) / n
                    self.readys[i] = True

    def run(self):
        # thread target
        while self.is_running:
            self.update()
            time.sleep(0.001)

    def start(self):
        # start thread
        if not self.is_running:
            self.thread = Thread(target=self.run)
            self.is_running = True
            self.thread.start()

    def stop(self):
        # stop thread
        self.is_running = False