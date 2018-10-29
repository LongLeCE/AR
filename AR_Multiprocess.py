from OpenGL.GLUT import *
from OpenGL.GLU import *
import cv2
from PIL import Image
import numpy as np
from glyph_modded_process import Glyphs
from objloader import *
from multiprocessing import Process, Queue
import pygame
from glyphfunctions_modded import *


class Webcam:
    def __init__(self, q):
        self.process = Process(target=self._update_frame, args=(q,))

    # create process for capturing images
    def start(self):
        self.process.start()

    def _update_frame(self, q):
        video_capture = cv2.VideoCapture(cv2.CAP_DSHOW)
        while True:
            current_frame = video_capture.read()[1]
            q.put(current_frame)
    #
    # # get the current frame
    # def get_current_frame(self):
    #     return q.get()


class SiftMatching:
    def __init__(self):
        self.current_frame = Queue()
        self.webcam = Webcam(self.current_frame)
        # self.webcam.start()
        self.sound_play = False

    def match(self, I2, q, name):
        while True:
            I1 = self.current_frame.get()
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


class Glyphs:
    s1 = SiftMatching()
    s2 = SiftMatching()
    s3 = SiftMatching()
    s4 = SiftMatching()
    s5 = SiftMatching()
    s6 = SiftMatching()
    q = Queue()
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


class OpenGLGlyphs:
    # constants
    INVERSE_MATRIX = np.array([[1.0, 1.0, 1.0, 1.0],
                               [-1.0, -1.0, -1.0, -1.0],
                               [-1.0, -1.0, -1.0, -1.0],
                               [1.0, 1.0, 1.0, 1.0]])

    def __init__(self):
        # initialise webcam and start thread
        self.current_frame = Queue()
        self.webcam = Webcam(self.current_frame)
        # self.webcam.start()

        # initialise glyphs
        self.glyphs = Glyphs()

        # initialise shapes
        self.chicken = None
        self.elephant = None
        self.cat = None
        self.octopus = None
        self.dog = None
        self.tank_t54 = None
        self.isShapeSwitch = False

        # initialise texture
        self.texture_background = None

        # initialise view matrix
        self.view_matrix = np.array([])

    def _init_gl(self, Width, Height):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(33.7, 1.3, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

        # assign shapes
        self.chicken = OBJ('chicken.obj')
        self.elephant = OBJ('elephant.obj')
        self.cat = OBJ('cat.obj')
        self.octopus = OBJ('octopus.obj')
        self.dog = OBJ('dog.obj')
        self.tank_t54 = OBJ('tank_t54.obj')

        # assign texture
        glEnable(GL_TEXTURE_2D)
        self.texture_background = glGenTextures(1)

    def _draw_scene(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # get image from webcam
        image = self.current_frame.get()

        # convert image to OpenGL texture format
        bg_image = cv2.flip(image, 0)
        bg_image = Image.fromarray(bg_image)
        ix = bg_image.size[0]
        iy = bg_image.size[1]
        bg_image = bg_image.tobytes("raw", "BGRX", 0, -1)

        # create background texture
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)

        # draw background
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glPushMatrix()
        glTranslatef(0.0, 0.0, -10.0)
        self._draw_background()
        glPopMatrix()

        # handle glyphs
        image = self._handle_glyphs(image)

        glutSwapBuffers()

    def _handle_glyphs(self, image):

        # attempt to detect glyphs
        glyphs = []

        try:
            glyphs = self.glyphs.detect(image)
        except Exception as ex:
            print(ex)

        if not glyphs:
            return

        for glyph in glyphs:

            rvecs, tvecs, glyph_name = glyph

            # build view matrix
            rmtx = cv2.Rodrigues(rvecs)[0]

            # if self.view_matrix.size == 0:
            self.view_matrix = np.array([[rmtx[0][0], rmtx[0][1], rmtx[0][2], tvecs[0]],
                                         [rmtx[1][0], rmtx[1][1], rmtx[1][2], tvecs[1]],
                                         [rmtx[2][0], rmtx[2][1], rmtx[2][2], tvecs[2]],
                                         [0.0, 0.0, 0.0, 1.0]])

            self.view_matrix = self.view_matrix * self.INVERSE_MATRIX

            self.view_matrix = np.transpose(self.view_matrix)

            # load view matrix and draw shape
            glPushMatrix()
            glLoadMatrixd(self.view_matrix)

            if self.glyphs.name == 'cat':
                glCallList(self.cat.gl_list)
            elif self.glyphs.name == 'chicken':
                glCallList(self.chicken.gl_list)
            elif self.glyphs.name == 'elephant':
                glCallList(self.elephant.gl_list)
            elif self.glyphs.name == 'octopus':
                glCallList(self.octopus.gl_list)
            elif self.glyphs.name == 'dog':
                glCallList(self.dog.gl_list)
            elif self.glyphs.name == 'tank_t54':
                glCallList(self.tank_t54.gl_list)

            glPopMatrix()

    def _draw_background(self):
        # draw background
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0)
        glVertex3f(-4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 1.0)
        glVertex3f(4.0, -3.0, 0.0)
        glTexCoord2f(1.0, 0.0)
        glVertex3f(4.0, 3.0, 0.0)
        glTexCoord2f(0.0, 0.0)
        glVertex3f(-4.0, 3.0, 0.0)
        glEnd()

    def _key_pressed(self, key, x, y):
        # toggle the shape switch
        self.isShapeSwitch = not self.isShapeSwitch

    def main(self):
        # setup and run OpenGL
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(1280, 960)
        glutInitWindowPosition(0, 0)
        self.window_id = glutCreateWindow("OpenGL Glyphs")
        glutDisplayFunc(self._draw_scene)
        glutIdleFunc(self._draw_scene)
        glutKeyboardFunc(self._key_pressed)
        self._init_gl(1280, 960)
        glutMainLoop()


# run an instance of OpenGL Glyphs
if __name__ == '__main__':
    openGLGlyphs = OpenGLGlyphs()
    openGLGlyphs.webcam.start()
    openGLGlyphs.glyphs.s1.webcam.start()
    openGLGlyphs.glyphs.s2.webcam.start()
    openGLGlyphs.glyphs.s3.webcam.start()
    openGLGlyphs.glyphs.s4.webcam.start()
    openGLGlyphs.glyphs.s5.webcam.start()
    openGLGlyphs.glyphs.s6.webcam.start()
    openGLGlyphs.glyphs.s1.start(openGLGlyphs.glyphs.I01, openGLGlyphs.glyphs.q, 'chicken')
    openGLGlyphs.glyphs.s2.start(openGLGlyphs.glyphs.I02, openGLGlyphs.glyphs.q, 'elephant')
    openGLGlyphs.glyphs.s3.start(openGLGlyphs.glyphs.I03, openGLGlyphs.glyphs.q, 'cat')
    openGLGlyphs.glyphs.s4.start(openGLGlyphs.glyphs.I04, openGLGlyphs.glyphs.q, 'octopus')
    openGLGlyphs.glyphs.s5.start(openGLGlyphs.glyphs.I05, openGLGlyphs.glyphs.q, 'dog')
    openGLGlyphs.glyphs.s6.start(openGLGlyphs.glyphs.I06, openGLGlyphs.glyphs.q, 'tank_t54')
    openGLGlyphs.glyphs.s1.initiate_sound()
    openGLGlyphs.main()
