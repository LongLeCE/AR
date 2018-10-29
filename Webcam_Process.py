import cv2
from multiprocessing import Process


class Webcam:

    def __init__(self, q):
        self.process = Process(target=self._update_frame, args=(q,))

    # create process for capturing images
    def start(self):
        self.process.start()

    def _update_frame(self, q):
        video_capture = cv2.VideoCapture(cv2.CAP_DSHOW)
        while True:
            if video_capture.isOpened():
                current_frame = video_capture.read()[1]
                q.put(current_frame)
    #
    # # get the current frame
    # def get_current_frame(self):
    #     return q.get()
