from glyphfunctions_modded import *
from Detect_Thread import SiftMatching
import cv2
import ctypes


# s1 = SiftMatching()
# s2 = SiftMatching()
# s3 = SiftMatching()
# s4 = SiftMatching()
# s5 = SiftMatching()
# s6 = SiftMatching()
# I01 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\1.jpg")
# I01 = cv2.resize(I01, (360, 240))
# I02 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\2.png")
# I02 = cv2.flip(cv2.resize(I02, (360, 240)), 1)
# I03 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\3.png")
# I03 = cv2.resize(I03, (360, 240))
# I04 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\4.png")
# I04 = cv2.resize(I04, (360, 240))
# I05 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\5.png")
# I05 = cv2.resize(I05, (360, 240))
# I06 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\6.png")
# I06 = cv2.resize(I06, (360, 240))
# s1.start(I01)
# s2.start(I02)
# s3.start(I03)
# s4.start(I04)
# s5.start(I05)
# s6.start(I06)
# s1.initiate_sound()


class Glyphs:
    QUADRILATERAL_POINTS = 4
    BLACK_THRESHOLD = 110
    WHITE_THRESHOLD = 150
    count = -46
    initial = 1
    corners_old = []
    name = 'None'

    def __init__(self, webcam):
        self.s1 = SiftMatching()
        self.s2 = SiftMatching()
        self.s3 = SiftMatching()
        self.s4 = SiftMatching()
        self.s5 = SiftMatching()
        self.s6 = SiftMatching()
        self.I01 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\1.jpg")
        self.I01 = cv2.resize(self.I01, (360, 240))
        self.I02 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\2.png")
        self.I02 = cv2.flip(cv2.resize(self.I02, (360, 240)), 1)
        self.I03 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\3.png")
        self.I03 = cv2.resize(self.I03, (360, 240))
        self.I04 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\4.png")
        self.I04 = cv2.resize(self.I04, (360, 240))
        self.I05 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\5.png")
        self.I05 = cv2.resize(self.I05, (360, 240))
        self.I06 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\6.png")
        self.I06 = cv2.resize(self.I06, (360, 240))
        self.s1.start(self.I01, webcam)
        self.s2.start(self.I02, webcam)
        self.s3.start(self.I03, webcam)
        self.s4.start(self.I04, webcam)
        self.s5.start(self.I05, webcam)
        self.s6.start(self.I06, webcam)
        self.s1.initiate_sound()
        # self.webcam = id(webcam)

    def stabilize_corners(self, old, new):
        moved = False
        for idx in range(len(new)):
            if abs(new[idx][0] - old[idx][0]) > 20 or abs(new[idx][1] - old[idx][1]) > 20:
                moved = True
                break
        if moved is True:
            return True, new
        else:
            return False, old

    def detect(self, image):

        glyphs = []

        if self.s1.glyph_found or self.s2.glyph_found or self.s3.glyph_found or self.s4.glyph_found or self.s5.glyph_found or self.s6.glyph_found:
            approx = []
            # mat = []
            line = []
            if self.s1.glyph_found:
                approx = self.s1.npts
                # mat = s1.M
                line = self.s1.move
                # approx = s1.npts.reshape(8, 2)
                if self.name != 'chicken':
                    self.name = 'chicken'
                    self.s1.stop_current_and_load_sound(self.name)
            elif self.s2.glyph_found:
                approx = self.s2.npts
                # mat = s2.M
                line = self.s2.move
                # approx = s2.npts.reshape(8, 2)
                if self.name != 'elephant':
                    self.name = 'elephant'
                    self.s1.stop_current_and_load_sound(self.name)
            elif self.s3.glyph_found:
                approx = self.s3.npts
                # mat = s3.M
                line = self.s3.move
                # approx = s3.npts.reshape(8, 2)
                if self.name != 'cat':
                    self.name = 'cat'
                    self.s1.stop_current_and_load_sound(self.name)
            elif self.s4.glyph_found:
                approx = self.s4.npts
                # mat = s4.M
                line = self.s4.move
                # approx = s4.npts.reshape(8, 2)
                if self.name != 'octopus':
                    self.name = 'octopus'
                    self.s1.stop_current_and_load_sound(self.name)
            elif self.s5.glyph_found:
                approx = self.s5.npts
                # mat = s5.M
                line = self.s5.move
                # approx = s5.npts.reshape(8, 2)
                if self.name != 'dog':
                    self.name = 'dog'
                    self.s1.stop_current_and_load_sound(self.name)
            elif self.s6.glyph_found:
                approx = self.s6.npts
                # mat = s6.M
                line = self.s6.move
                # approx = s6.npts.reshape(8, 2)
                if self.name != 'tank_t54':
                    self.name = 'tank_t54'
                    self.s1.stop_current_and_load_sound(self.name)

            # if self.initial == 1:
            #     self.corners_old = approx
            #     self.initial = 0
            # # print(self.stabilize_corners(self.corners_old, approx))
            # ret, approx = self.stabilize_corners(self.corners_old, approx)
            # if ret is True:
            #     self.corners_old = approx
            #     print(self.corners_old)

            # print(approx)
            # approx = cv2.perspectiveTransform(approx.reshape(-1, 1, 2), mat).reshape(4, 2)

            if self.count == 45:
                self.count = -46
            self.count += 1

            # for i in range(4):
            #     cv2.circle(image, (approx[i][0], approx[i][1]), 5, (0, 0, 255))
            # cv2.imshow("img", image)
            rvecs, tvecs = get_vectors(image, approx, self.count, line)
            # rvecs, tvecs = get_vectors(image, approx, ctypes.cast(self.webcam, ctypes.py_object).value.frame_counter, line)
            glyphs.append([rvecs, tvecs, self.name])

        else:
            self.name = 'silent'
            self.s1.stop_current_and_load_sound(self.name)

        return glyphs
