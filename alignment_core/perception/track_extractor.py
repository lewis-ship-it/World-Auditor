import cv2
import numpy as np


def extract_track_centerline(image, threshold=120, downsample=8):

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    _, binary = cv2.threshold(
        gray,
        threshold,
        255,
        cv2.THRESH_BINARY_INV
    )

    kernel = np.ones((5,5), np.uint8)

    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours,_ = cv2.findContours(
        binary,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )

    if len(contours) == 0:
        return None

    contour = max(contours, key=cv2.contourArea)

    pts = contour[:,0,:]

    pts = pts[::downsample]

    return pts