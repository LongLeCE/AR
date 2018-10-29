import cv2
from multiprocessing import Process, Queue
from Webcam_Process import Webcam
import numpy as np
import pygame


class SiftMatching:
    def __init__(self):
        self.current_frame = Queue()
        self.webcam = Webcam(self.current_frame)
        # self.webcam.start()
        self.sound_play = False

    def match(self, I2, q, name):
        while True:
            I1 = self.current_frame.get()
            # print(I1)
            # print(I1.shape)
            gray1 = cv2.cvtColor(I1, cv2.COLOR_RGB2GRAY)
            sift = cv2.xfeatures2d.SIFT_create()
            kpt1, des1 = sift.detectAndCompute(gray1, None)
            gray2 = cv2.cvtColor(I2, cv2.COLOR_RGB2GRAY)
            kpt2, des2 = sift.detectAndCompute(gray2, None)
            # Matching Brute force
            bf = cv2.BFMatcher_create()
            matches = bf.knnMatch(des2, des1, 2)  # knn: k nearest neighbor
            # Choose good matches
            good = []
            new_good = []
            for m, n in matches:
                if m.distance < 0.4 * n.distance:
                    good.append([m])
                    new_good.append(m)
            if len(good) > 3:
                srcPoints = np.float32([kpt2[m.queryIdx].pt for m in new_good]).reshape(-1, 1, 2)
                dstPoints = np.float32([kpt1[m.trainIdx].pt for m in new_good]).reshape(-1, 1, 2)
                M, H = cv2.findHomography(srcPoints, dstPoints)
                w = gray2.shape[1]-1
                h = gray2.shape[0]-1
                n_corners = np.float32([[0, h], [w, h], [w, 0], [0, 0]]).reshape(-1, 1, 2)
                # n_corners = np.float32([[0, h], [w/2, h], [w, h], [w, h/2], [w, 0], [w/2, 0], [0, 0], [0, h/2]]).reshape(-1, 1, 2)
                if M is not None:
                    q.put([cv2.perspectiveTransform(n_corners, M), True, name])
                    # self.npts = cv2.perspectiveTransform(n_corners, M)
                    # print(self.npts)
                    # # self.npts = np.int32(self.npts)
                else:
                    q.put([[], True, name])
            else:
                q.put([[], False, name])

    def start(self, src_img, q, name):
        Process(target=self.match, args=(src_img, q, name)).start()

    def initiate_sound(self):
        pygame.mixer.init()
        pygame.mixer.music.load("D:\\Python\\Lesson 1\\silent.mp3")
        pygame.mixer.music.play(-1)

    def stop_current_and_load_sound(self, sound):
        pygame.mixer.music.stop()
        pygame.mixer.music.load("D:\\Python\\Lesson 1\\" + sound + ".mp3")
        pygame.mixer.music.play(-1)
