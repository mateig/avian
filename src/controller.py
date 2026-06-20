# drone/controller.py
from threading import Lock

import numpy as np
from pos_filter import PositionFilter

RC_CENTER_ROLL = 1510
RC_CENTER_PITCH = 1500
RC_CENTER_THROTTLE = 1265
RC_CENTER_YAW = 1500

RC_RANGE_ROLL = 20
RC_RANGE_PITCH = 20
RC_RANGE_THROTTLE = 30
RC_RANGE_YAW = 0

KP = 3.0
KD = 20.0

MIN_ALTITUDE = 0.0
MAX_ALTITUDE = 1.5
MIN_X = -1.0
MAX_X = 1.0
MIN_Y = -1
MAX_Y = 1.0

POSITION_DEADBAND = 0.01

RC_RATE_LIMIT_ROLL = 10
RC_RATE_LIMIT_PITCH = 10
RC_RATE_LIMIT_THROTTLE = 10
RC_RATE_LIMIT_YAW = 0


class Controller:
    def __init__(self, pos_filter: PositionFilter, dt: float = 1 / 30) -> None:
        self.pf = pos_filter
        self.dt = dt
        self.enabled = False

        self.target = np.array([0.0, 0.0, 0.0])
        self.prev_rc_roll = RC_CENTER_ROLL
        self.prev_rc_pitch = RC_CENTER_PITCH
        self.prev_rc_throttle = RC_CENTER_THROTTLE
        self.prev_rc_yaw = RC_CENTER_YAW
        self.lock = Lock()

    def enable(self) -> None:
        if self.pf.is_ready():
            pos, _ = self.pf.get_state()
            self.set_target(pos[0], pos[1], pos[2])
            self._reset()
            self.enabled = True
        else:
            print("Cannot enable PositionController: PositionFilter not ready")

    def is_enabled(self) -> bool:
        return self.enabled

    def disable(self) -> None:
        self.enabled = False
        self._reset()

    def set_target(self, x: float, y: float, z: float) -> None:
        with self.lock:
            self.target[0] = np.clip(x, MIN_X, MAX_X)
            self.target[1] = np.clip(y, MIN_Y, MAX_Y)
            self.target[2] = np.clip(z, MIN_ALTITUDE, MAX_ALTITUDE)
        self._reset()

    def update(self) -> tuple[int, int, int, int]:
        if not self.enabled or not self.pf.is_ready():
            return (
                RC_CENTER_ROLL,
                RC_CENTER_PITCH,
                RC_CENTER_THROTTLE,
                RC_CENTER_YAW,
            )

        pos, vel = self.pf.get_state()
        with self.lock:
            pos_err = self.target - pos
            pos_err = self._apply_deadband(pos_err, POSITION_DEADBAND)

        roll_raw = RC_CENTER_ROLL + (pos_err[0] * KP - vel[0] * KD) * RC_RANGE_ROLL
        pitch_raw = RC_CENTER_PITCH + (pos_err[1] * KP - vel[1] * KD) * RC_RANGE_PITCH
        throttle_raw = RC_CENTER_THROTTLE
        yaw_raw = RC_CENTER_YAW

        rc_roll, rc_pitch, rc_throttle, rc_yaw = self._apply_safety_limits(
            roll_raw, pitch_raw, throttle_raw, yaw_raw
        )

        self.prev_rc_roll = rc_roll
        self.prev_rc_pitch = rc_pitch
        self.prev_rc_throttle = rc_throttle
        self.prev_rc_yaw = rc_yaw

        return rc_roll, rc_pitch, rc_throttle, rc_yaw

    def _apply_deadband(self, error: np.ndarray, deadband: float) -> np.ndarray:
        return np.where(np.abs(error) < deadband, 0.0, error)

    def _apply_safety_limits(
        self, roll: float, pitch: float, throttle: float, yaw: float
    ) -> tuple[int, int, int, int]:
        # Rate limits
        roll = np.clip(
            roll,
            self.prev_rc_roll - RC_RATE_LIMIT_ROLL,
            self.prev_rc_roll + RC_RATE_LIMIT_ROLL,
        )
        pitch = np.clip(
            pitch,
            self.prev_rc_pitch - RC_RATE_LIMIT_PITCH,
            self.prev_rc_pitch + RC_RATE_LIMIT_PITCH,
        )
        throttle = np.clip(
            throttle,
            self.prev_rc_throttle - RC_RATE_LIMIT_THROTTLE,
            self.prev_rc_throttle + RC_RATE_LIMIT_THROTTLE,
        )
        yaw = np.clip(
            yaw,
            self.prev_rc_yaw - RC_RATE_LIMIT_YAW,
            self.prev_rc_yaw + RC_RATE_LIMIT_YAW,
        )

        # Hard limits
        roll = int(
            np.clip(
                roll, RC_CENTER_ROLL - RC_RANGE_ROLL, RC_CENTER_ROLL + RC_RANGE_ROLL
            )
        )
        pitch = int(
            np.clip(
                pitch,
                RC_CENTER_PITCH - RC_RANGE_PITCH,
                RC_CENTER_PITCH + RC_RANGE_PITCH,
            )
        )
        throttle = int(
            np.clip(
                throttle,
                RC_CENTER_THROTTLE - RC_RANGE_THROTTLE,
                RC_CENTER_THROTTLE + RC_RANGE_THROTTLE,
            )
        )
        yaw = int(
            np.clip(yaw, RC_CENTER_YAW - RC_RANGE_YAW, RC_CENTER_YAW + RC_RANGE_YAW)
        )

        return roll, pitch, throttle, yaw

    def _reset(self) -> None:
        with self.lock:
            self.prev_rc_roll = RC_CENTER_ROLL
            self.prev_rc_pitch = RC_CENTER_PITCH
            self.prev_rc_throttle = RC_CENTER_THROTTLE
            self.prev_rc_yaw = RC_CENTER_YAW
