# research

Offline analysis notebooks. These produce the figures and numbers behind the design
choices in `../src`. They read logged recordings from the project's `data/` directory,
which lives outside this repo; point `research/data` at it (a copy or a local link) before
running the notebooks.

- `calib.ipynb` — camera calibration: intrinsics, reprojection error, and bootstrapped
  uncertainty on the focal length and principal point.
- `pnp.ipynb` — PnP analysis: per-flight detection rate, dropouts, reprojection error, and
  effective measurement range.
- `vio.ipynb` — the markerless feature-based visual-odometry attempt (FAST + Lucas-Kanade +
  essential-matrix recovery) that did not produce usable pose and was abandoned.
- `pid.ipynb` — controller tuning and closed-loop position-hold analysis.

Supporting files:

- `capture_stream.py` — captures a camera stream for offline processing.
- `utils.py` — shared helpers for the notebooks.
- `figures/` — exported plots and videos used in the manuscript and presentation.

## Running

```bash
pip install -r requirements.txt
jupyter lab
```
