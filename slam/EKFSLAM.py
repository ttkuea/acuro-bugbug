import cv2
import numpy as np


def F(x):
    return np.concatenate((np.eye(3), np.zeros((3, x))), axis=1)

class EKFSLAM:

    def __init__(self, n_aruco=4):
        self.n_aruco = 4

        self.vector_function = None

        # initialize matrices
        self.mu = np.zeros((1, 3 + 2 * self.n_aruco), float)
        self.sigma = np.zeros((3 + 2 * self.n_aruco, 3 + 2 * self.n_aruco), float)
        for i in range(3, 3 + 2 * self.n_aruco):
            self.sigma[i, i] = np.inf

    def bindInput(self, vector_function):
        self.vector_function = vector_function

    def getMatrices(self):
        return self.mu, self.sigma

    def g(self, v, w, dt):

        theta = self.mu[0, 2]
        U = np.array([
            [- v / w * np.sin(theta) + v / w * np.sin(theta + w * dt)],
            [v / w * np.cos(theta) - v / w * np.cos(theta + w * dt)],
            [w * dt]
        ])
        return self.mu + (F(2 * self.n_aruco).T @ U)

    def G(self, v, w, dt):
        theta = self.mu[0, 2]

        G = np.eye(3 + 2 * self.n_aruco)
        Gx = np.array([
            [1, 0, - v / w * np.cos(theta) + v / w * np.cos(theta + w * dt)],
            [0, 1, - v / w * np.sin(theta) + v / w * np.sin(theta + w * dt)],
            [0, 0, 1]
        ])
        G[0:2, 0:2] = G[0:2, 0:2] @ Gx
        return G

    def update(self):

        # 1
        v, w, dt = self.vector_function()
        mu_ = self.g(v, w, dt)

        # 2
        G = self.G(v, w, dt)
        # R = self.R
        sigma_ = (G @ (self.sigma @ G.T)) #+ R

        # 3
        H = self.H()
        Q = self.Q()
        K = sigma_ @ (H.T @ (np.linalg.inv((H @ (sigma_ @ H.T)) + Q)))

        # 4
        z = self.z()
        self.mu = mu_ + K @ (z - self.h(mu_))

        # 5
        self.sigma_ = (I - K @ H) @ sigma_
