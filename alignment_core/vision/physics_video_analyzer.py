import cv2
import numpy as np


class PhysicsVideoAnalyzer:

    def __init__(self):
        pass


    def estimate_motion(self, video_path):

        cap = cv2.VideoCapture(video_path)

        prev_gray = None

        velocities = []

        fps = cap.get(cv2.CAP_PROP_FPS)

        while True:

            ret, frame = cap.read()

            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if prev_gray is not None:

                flow = cv2.calcOpticalFlowFarneback(
                    prev_gray,
                    gray,
                    None,
                    0.5,
                    3,
                    15,
                    3,
                    5,
                    1.2,
                    0
                )

                mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])

                avg_motion = np.mean(mag)

                velocity = avg_motion * fps

                velocities.append(velocity)

            prev_gray = gray

        cap.release()

        return velocities