# src

Onboard flight code. Runs on the Raspberry Pi Zero 2 W companion computer, alongside a
Betaflight flight controller connected over UART.

`run.py` starts three threads, each targeting 30 Hz:

- **vision** (`vision.py`) — captures frames from the Pi Camera, detects the chessboard
  corners, and recovers camera pose with `solvePnP` (IPPE planar solver), transformed into
  the drone body frame.
- **position filter** (`pos_filter.py`) — a 3D constant-velocity Kalman filter that smooths
  the vision measurements and coasts on the velocity model when measurements drop out.
- **flight controller** (`fc.py`) — reads the pilot's RC channels over MSP, mixes in the
  autopilot's roll/pitch when the autonomy switch is engaged, and writes the result back to
  the flight controller. Throttle and yaw are always passed through from the pilot.

Supporting modules:

- `controller.py` — PD position law (KP = 3.0, KD = 20.0) mapping position error and
  velocity to roll/pitch RC offsets, with a deadband, per-step rate limits, and hard
  authority limits.
- `msp.py` — minimal MSP (MultiWii Serial Protocol) client for reading RC/IMU/attitude and
  writing `MSP_SET_RAW_RC`.
- `calib_data.npz` — camera intrinsics (`mtx`, `dist`) loaded by `vision.py`.

## Running

On the Pi, from this directory:

```bash
pip install -r requirements.txt
python run.py
```

`picamera2` ships with Raspberry Pi OS and is not installed via pip; the rest are in
`requirements.txt`. The code loads `calib_data.npz` from the working directory, so run it
from `src/`. Send SIGINT (Ctrl-C) to shut down cleanly.
