import cv2
import numpy as np

def extract_lane_center(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _, binary = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY_INV)

    h, w = binary.shape

    # focus bottom half (road area)
    roi = binary[int(h*0.5):, :]

    moments = cv2.moments(roi)

    if moments["m00"] == 0:
        return None

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"]) + int(h*0.5)

    return (cx, cy)