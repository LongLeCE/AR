from glyphfunctions_modded import *
from Detect_Process import SiftMatching
import multiprocessing as mp


class Glyphs:
    s1 = SiftMatching()
    s2 = SiftMatching()
    s3 = SiftMatching()
    s4 = SiftMatching()
    s5 = SiftMatching()
    s6 = SiftMatching()
    q = mp.JoinableQueue()
    q.put([], False, 'silent')
    I01 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\1.jpg")
    I01 = cv2.resize(I01, (360, 240))
    I02 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\2.png")
    I02 = cv2.flip(cv2.resize(I02, (360, 240)), 1)
    I03 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\3.png")
    I03 = cv2.resize(I03, (360, 240))
    I04 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\4.png")
    I04 = cv2.resize(I04, (360, 240))
    I05 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\5.png")
    I05 = cv2.resize(I05, (360, 240))
    I06 = cv2.imread("D:\\Python\\Lesson 1\\Detect\\6.jpg")
    I06 = cv2.resize(I06, (360, 240))

    QUADRILATERAL_POINTS = 4
    BLACK_THRESHOLD = 110
    WHITE_THRESHOLD = 150
    count = -46
    initial = 1
    corners_old = []
    name = 'silent'

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

        if self.q.get()[1]:
            approx = self.q.get()[0].reshape(4, 2)
            if self.name != self.q.get()[2]:
                self.name = self.q.get()[2]
                self.s1.stop_current_and_load_sound(self.name)

            if self.initial == 1:
                self.corners_old = approx
                self.initial = 0
            # print(self.stabilize_corners(self.corners_old, approx))
            ret, approx = self.stabilize_corners(self.corners_old, approx)
            if ret is True:
                self.corners_old = approx
                # print(self.corners_old)
            # print(approx)

            if self.count == 45:
                self.count = -46
            self.count += 1

            # for i in range(4):
            #     cv2.circle(image, (approx[i][0], approx[i][1]), 5, (0, 0, 255))
            # cv2.imshow("img", image)
            rvecs, tvecs = get_vectors(image, approx, self.count)
            glyphs.append([rvecs, tvecs, self.name])

        else:
            self.name = 'silent'
            self.s1.stop_current_and_load_sound(self.name)

        return glyphs
