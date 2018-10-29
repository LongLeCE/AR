from OpenGL.GLUT import *
from OpenGL.GLU import *
import cv2
from PIL import Image
import numpy as np
from Webcam_Process import Webcam
from glyph_modded_process import Glyphs
from objloader import *
from multiprocessing import Queue


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
