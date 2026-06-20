# drone/pos_filter.py
import time
from threading import Event, Lock

import numpy as np

PROC_NOISE = 0.01
MEAS_NOISE = 0.5
OFFSET_SAMPLES = 10


class PositionFilter:
    def __init__(self, vb_lock: Lock, vision_buffer: list, dt: float = 1 / 30) -> None:
        self.vb_lock = vb_lock  # vision buffer lock
        self.vision_buffer = vision_buffer

        self.lock = Lock()

        self.pos = None
        self.vel = None
        self.P = np.eye(3) * 1.0  # covariance
        self.Q = np.eye(3) * PROC_NOISE  # process noise
        self.R = np.eye(3) * MEAS_NOISE  # measurement noise
        self.dt = dt

        self.ready = False
        self.start_poses = []
        self.offset = np.array([0.0, 0.0, 0.0])

        self.storage = []

    def start(self, shutdown: Event) -> None:
        print("Position filter calibrating...")
        while not shutdown.is_set() and len(self.start_poses) < OFFSET_SAMPLES:
            with self.vb_lock:
                if len(self.vision_buffer) > 0:
                    meas_time, measurement = self.vision_buffer.pop(0)
                    if measurement is not None:
                        self.start_poses.append(measurement)
            time.sleep(0.01)

        with self.lock:
            if len(self.start_poses) == OFFSET_SAMPLES:
                self.offset = np.mean(self.start_poses, axis=0)
                self.pos = self.offset.copy()
                self.vel = np.zeros(3)
                self.ready = True
        print(f"Position filter calibrated. Offset: {self.offset}")

        self.loop(shutdown)
        self.store_output("pf_output.npz")

    def loop(self, shutdown: Event) -> None:
        while not shutdown.is_set():
            with self.vb_lock:
                if len(self.vision_buffer) > 0:
                    meas_time, measurement = self.vision_buffer.pop(0)
                else:
                    measurement = None

            with self.lock:
                self.update(measurement)

            self.storage.append((time.time(), self.get_state()))
            time.sleep(self.dt)

    def update(self, measurement: np.ndarray | None) -> None:
        if measurement is None:
            if self.pos is not None and self.vel is not None:
                self.pos += self.vel * self.dt
                self.P += self.Q
        else:
            if self.pos is None:
                self.pos = measurement
                self.vel = np.zeros(3)
            else:
                pred_pos = self.pos + self.vel * self.dt
                self.P += self.Q

                K = self.P @ np.linalg.inv(self.P + self.R)
                innovation = measurement - pred_pos

                self.pos = pred_pos + K @ innovation
                self.vel = self.vel + (K @ innovation) / self.dt * 0.1
                self.P = (np.eye(3) - K) @ self.P

    def is_ready(self) -> bool:
        with self.lock:
            return self.ready

    def get_state(self) -> tuple[np.ndarray, np.ndarray]:
        with self.lock:
            return self.pos - self.offset, self.vel

    def store_output(self, filename: str) -> None:
        times, poses, vels = [], [], []
        for t, (p, v) in self.storage:
            if p is not None and v is not None:
                times.append(t)
                poses.append(p)
                vels.append(v)
            else:
                times.append(t)
                poses.append(np.array([np.nan, np.nan, np.nan]))
                vels.append(np.array([np.nan, np.nan, np.nan]))
        times = np.array(times)
        poses = np.array(poses)
        vels = np.array(vels)
        np.savez(filename, times=times, poses=poses, vels=vels)
