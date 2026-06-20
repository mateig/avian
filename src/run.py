# drone/run.py
import signal
from concurrent.futures import ThreadPoolExecutor
from threading import Event, Lock

from fc import FlightController
from controller import Controller
from pos_filter import PositionFilter
from vision import Vision

shutdown = Event()


def signal_handler(sig, frame):
    print("Shutdown signal received")
    shutdown.set()


def main():
    vb_lock = Lock()
    vision_buffer = []

    vision = Vision(vb_lock, vision_buffer)
    pf = PositionFilter(vb_lock, vision_buffer)
    fc = FlightController(pf)

    # image processing thread
    # position estimation thread
    # drone interface thread

    with ThreadPoolExecutor(max_workers=3) as executor:
        pv_fut = executor.submit(vision.start, shutdown)
        pf_fut = executor.submit(pf.start, shutdown)
        fc_fut = executor.submit(fc.start, shutdown)

        pv_fut.result()
        pf_fut.result()
        fc_fut.result()

    fc.close()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    main()
