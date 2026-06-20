# AVIAN — code

Onboard flight code and analysis notebooks for AVIAN, an indoor GPS-denied position-hold
quadrotor built on a Raspberry Pi Zero 2 W, a Pi Camera, and a single printed chessboard
fiducial. See the top-level project summary for the full write-up.

## Layout

- `src/` — onboard flight code that runs on the Pi: vision, Kalman filter, PD controller,
  and the MSP flight-controller interface.
- `research/` — analysis notebooks for calibration, PnP, visual odometry, and controller
  tuning. `research/data` is a symlink to the project's `data/` directory.

Each folder has its own README.

## Overview

The system localizes the drone indoors from a single camera viewing a printed chessboard.
Three threads run on the Pi at 30 Hz: a vision thread recovers camera pose from the marker
with PnP, a Kalman filter smooths it into a position estimate, and a PD controller writes
roll/pitch corrections to the Betaflight flight controller over MSP. Throttle and yaw are
passed through from the pilot, who can override autonomy at any time via an RC switch.

The notebooks in `research/` cover the offline analysis behind these choices: camera
calibration, PnP detection rates and reprojection error, the markerless visual-odometry
attempt that was abandoned, and controller tuning.
