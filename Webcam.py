import cv2
from threading import Thread
import time


class Webcam:

    def __init__(self):
        # print(cv2.VideoCapture(cv2.CAP_DSHOW).read())
        # print(cv2.CAP_DSHOW)
        self.video_capture = cv2.VideoCapture(cv2.CAP_DSHOW)
        # On opencv-contrib-python < 3.4: cv2.VideoCapture(0) is ok
        self.current_frame = self.video_capture.read()[1]
        # self.frame_counter = 0
        # self.t1 = 0
        # self.t2 = 0

    # create thread for capturing images
    def start(self):
        Thread(target=self._update_frame, args=()).start()

    def _update_frame(self):
        while True:
            # self.t2 = time.perf_counter()
            # print(1 / (self.t2 - self.t1))
            # self.t1 = self.t2
            self.current_frame = self.video_capture.read()[1]
            # if self.frame_counter == 45:
            #     self.frame_counter = -46
            # self.frame_counter += 1

    # get the current frame
    def get_current_frame(self):
        return self.current_frame
