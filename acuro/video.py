import cv2
import time

from threading import Thread


class Video:
    def __init__(self, input=0, fps=60):
        self.cap = cv2.VideoCapture(input)
        # self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.fps = fps
        self.image = None

        self.thread = None
        self.is_running = False
        self.start_time = -1
        self.last_update = -1

    def getImage(self):
        return self.image

    def run(self):
        while self.is_running:
            now = time.time()
            dt = -1
            if self.last_update != -1:
                dt = (now - self.last_update)

            if dt == -1 or dt > 1 / self.fps:
                ret, image = self.cap.read()
                if ret:
                    self.image = image
                    self.last_update = now
            
            time.sleep(0.001)
        self.cap.release()

    def start(self):
        if not self.is_running:
            self.thread = Thread(target=self.run)
            self.is_running = True
            self.thread.start()

    def stop(self):
        self.is_running = False