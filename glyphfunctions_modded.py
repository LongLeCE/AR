import cv2
import numpy as np
import math
import time


# def order_points(points):
#     s = points.sum(axis=1)
#     diff = np.diff(points, axis=1)
#
#     ordered_points = np.zeros((4, 2), dtype="float32")
#
#     ordered_points[0] = points[np.argmin(s)]
#     ordered_points[2] = points[np.argmax(s)]
#     ordered_points[1] = points[np.argmin(diff)]
#     ordered_points[3] = points[np.argmax(diff)]
#
#     return ordered_points

def get_vectors(image, points, count, line):
    with np.load('webcam_calibration_ouput.npz') as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
    mtx[0, 0] = 720
    mtx[1, 1] = 720
    distance = math.sqrt((line[0] - line[2])**2 + (line[1] - line[3])**2)
    cos = (line[2] - line[0]) / distance
    sin = (line[3] - line[1]) / distance
    mtx[0, 2] = image.shape[1] / 2 - image.shape[1] * cos / 300 * count
    mtx[1, 2] = image.shape[0] / 2 - image.shape[0] * sin / 300 * count

    imgp = np.array(points, dtype="float32")

    # objp = np.array([[0., 1., 0.], [0., 1., 0.5], [0., 1., 1.], [0., 0.5, 1.], [0., 0., 1.], [0., 0., 0.5], [0., 0., 0.], [0., 0.5, 0.], [0., 0.5, 0.5]], dtype="float32")
    objp = np.array([[-0.5, 0, 0], [-0.5, 0, 1], [0.5, 0, 1], [0.5, 0, 0]], dtype="float32")
    # objp = np.array([[0.5, 0, 0], [0.5, 0, 1], [-0.5, 0, 1], [-0.5, 0, 0]], dtype="float32")
    # objp = np.array([[0.5, 1., 0], [0.5, 1., 0.5], [0.5, 1., 1], [0., 1., 1.], [-0.5, 1., 1], [-0.5, 1., 0.5], [-0.5, 1., 0], [0., 1., 0.]], dtype="float32")

    _, rvecs, tvecs = cv2.solvePnP(objp.reshape(-1, 1, 3), imgp.reshape(-1, 1, 2), mtx, dist, flags=cv2.SOLVEPNP_UPNP, useExtrinsicGuess=False)
    # time1 = time.perf_counter()
    # _, rvecs, tvecs, _ = cv2.solvePnPRansac(objp.reshape(-1, 1, 3), imgp.reshape(-1, 1, 2), mtx, dist, useExtrinsicGuess=False, flags=cv2.SOLVEPNP_UPNP)
    # time2 = time.perf_counter()
    # print((time2 - time1) * 1000000)

    return rvecs, tvecs
