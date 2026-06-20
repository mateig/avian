# utils.py
import av
import cv2
import numpy as np
from IPython.display import clear_output
from matplotlib import pyplot as plt


def load_video(path: str, gray: bool = False) -> np.ndarray:
    cap = cv2.VideoCapture(path)
    frames = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if gray:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frames.append(frame)
    cap.release()
    return np.array(frames).reshape(-1, *frames[0].shape)


def play_video(video):
    for n, frame in enumerate(video):
        clear_output(wait=True)
        plt.figure(figsize=(10, 6))
        plt.title(f"Frame {n+1}/{len(video)}")
        plt.imshow(frame, cmap="gray", vmin=np.min(video), vmax=np.max(video))
        plt.show()


def save_video(path: str, frames: list | np.ndarray, fps: int = 30) -> None:
    if isinstance(frames, list):
        frames = np.array(frames)
    height, width = frames.shape[1], frames.shape[2]

    container = av.open(path, mode="w", format="mp4")
    stream = container.add_stream("h264", rate=fps)
    stream.width = width
    stream.height = height
    stream.pix_fmt = "yuv420p"

    for f in frames:
        if f.ndim == 2:
            frame = av.VideoFrame.from_ndarray(f, format="gray")
        else:
            frame = av.VideoFrame.from_ndarray(f, format="bgr24")

        for packet in stream.encode(frame):
            container.mux(packet)

    for packet in stream.encode():
        container.mux(packet)
    container.close()
