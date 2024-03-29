import cv2
from threading import Thread
from Webcam import Webcam
import numpy as np
import pygame

def stabilize_corners(old, new):
    moved = False
    for idx in range(len(new)):
        if abs(new[idx][0] - old[idx][0]) > 10 or abs(new[idx][1] - old[idx][1]) > 10:
            moved = True
            break
    if moved is True:
        return True, new
    else:
        return False, old


class SiftMatching:
    def __init__(self):
        # self.webcam = Webcam()
        # self.webcam.start()
        self.glyph_found = False
        self.npts = []
        self.move = []
        self.sound_play = False
        self.initial = 1
        self.corners_old = []
        # self.M = []

    def match(self, _, I2, webcam):
        # SIFT and SURF not included in opencv-contrib-python >= 3.4.3.18
        sift = cv2.xfeatures2d.SIFT_create()
        gray2 = cv2.cvtColor(I2, cv2.COLOR_RGB2GRAY)
        kpt2, des2 = sift.detectAndCompute(gray2, None)
        bf = cv2.BFMatcher_create()
        while True:
            # I1 = self.webcam.get_current_frame()
            I1 = webcam.get_current_frame()
            gray1 = cv2.cvtColor(I1, cv2.COLOR_RGB2GRAY)
            kpt1, des1 = sift.detectAndCompute(gray1, None)
            # Matching Brute force
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
                # print(srcPoints)
                # print(dstPoints)
                M, H = cv2.findHomography(srcPoints, dstPoints)
                w = gray2.shape[1] - 1
                h = gray2.shape[0] - 1
                n_corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
                # moving_line = np.float32([[0, h / 2], [w, h / 2]]).reshape(-1, 1, 2)
                # print(n_corners)
                # n_corners = np.float32([[0, 0], [w/2, 0], [w, 0], [w, h/2], [w, h], [w/2, h], [0, h], [0, h/2], [w/2, h/2]]).reshape(-1, 1, 2)
                if M is not None:
                    self.npts = cv2.perspectiveTransform(n_corners, M).reshape(4, 2)
                    self.move = np.float32([(self.npts[0]+self.npts[3])/2, (self.npts[1]+self.npts[2])/2]).reshape(-1)
                    if self.initial == 1:
                        self.corners_old = self.npts
                        self.initial = 0
                    ret, self.npts = stabilize_corners(self.corners_old, self.npts)
                    if ret is True:
                        self.corners_old = self.npts
                    # print(self.move)
                    # print(self.move[0])
                    # self.M, _ = cv2.findHomography(self.npts.reshape(-1, 1, 2), cv2.perspectiveTransform(n_corners, M))
                    # print(self.npts)
                    # self.npts = np.int32(self.npts)
                self.glyph_found = True
            else:
                self.glyph_found = False

    def start(self, src_img, current_frame):
        Thread(target=self.match, args=(None, src_img, current_frame)).start()

    def initiate_sound(self):
        pygame.mixer.init()
        pygame.mixer.music.load("D:\\Python\\Lesson 1\\silent.mp3")
        pygame.mixer.music.play(-1)

    def stop_current_and_load_sound(self, sound):
        pygame.mixer.music.stop()
        pygame.mixer.music.load("D:\\Python\\Lesson 1\\" + sound + ".mp3")
        pygame.mixer.music.play(-1)
