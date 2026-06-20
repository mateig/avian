# drone/vision.py
import time
from threading import Event, Lock

import cv2
import numpy as np
from picamera2 import Picamera2

CB_ROWS, CB_COLS = 3, 5
CB_SQUARE_SIZE = 0.070
CB_ROT = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
CB_DETECT_SCALE = 0.75

SENSOR_WIDTH, SENSOR_HEIGHT = 2304, 1296
VIDEO_WIDTH, VIDEO_HEIGHT = 1280, 720
FRAME_RATE = 30

calib_data = np.load("calib_data.npz")
calib_mtx = calib_data["mtx"]
calib_dist = calib_data["dist"]

objp = np.empty((CB_ROWS * CB_COLS, 3), np.float32)
for i in range(CB_ROWS):
    for j in range(CB_COLS):
        objp[i * CB_COLS + j] = [j * CB_SQUARE_SIZE, i * CB_SQUARE_SIZE, 0]

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


class Vision:
    def __init__(self, lock: Lock, buffer: list) -> None:
        self.vb_lock = lock
        self.buffer = buffer

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (VIDEO_WIDTH, VIDEO_HEIGHT)},
            sensor={
                "output_size": (SENSOR_WIDTH, SENSOR_HEIGHT),
                "bit_depth": 10,
            },
            controls={
                "FrameDurationLimits": (int(1e6 / FRAME_RATE), int(1e6 / FRAME_RATE))
            },
        )
        self.picam2.configure(config)
        self.picam2.start()

    def start(self, shutdown: Event) -> None:
        while not shutdown.is_set():
            frame = self.picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cam_pos = self.get_cam_pos(
                gray,
                objp,
                (CB_COLS, CB_ROWS),
                calib_mtx,
                calib_dist,
                criteria,
                CB_DETECT_SCALE,
            )
            with self.vb_lock:
                self.buffer.append((time.time(), cam_pos))

    def get_cam_pos(
        self, img, objp, cb_size, calib_mtx, calib_dist, criteria, scale=1.0
    ) -> np.ndarray | None:
        scaled = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

        ret, corners = cv2.findChessboardCorners(
            scaled, cb_size, None, cv2.CALIB_CB_FAST_CHECK
        )
        if ret:
            corners = corners / scale
            corners_sb = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
            if corners_sb[0][0][0] > corners_sb[-1][0][0]:
                corners_sb = corners_sb[::-1]

            _, rvec, tvec = cv2.solvePnP(
                objp, corners_sb, calib_mtx, calib_dist, flags=cv2.SOLVEPNP_IPPE
            )

            R, _ = cv2.Rodrigues(rvec)
            cam_pos = CB_ROT @ (-R.T @ tvec)

            return cam_pos.flatten()
        return None
