# drone/fc.py
from threading import Event

from msp import MSP
from controller import Controller
from pos_filter import PositionFilter
import time

MSP_LATENCY = 0.01


class FlightController:
    def __init__(self, pos_filter: PositionFilter, dt: float = 1 / 30) -> None:
        self.pos_filter = pos_filter
        self.pos_con = Controller(pos_filter, dt)
        self.msp = MSP()
        self.dt = dt

        self.last_human_roll = 1500
        self.last_human_pitch = 1500
        self.last_human_throttle = 1000
        self.last_human_yaw = 1500

        self.autopilot_flag = False
        self.autopilot_ready = False

    def start(self, shutdown: Event) -> None:
        while not shutdown.is_set():
            curr_rc = self.msp.get_rc()

            if curr_rc["aux3"] == 1000:  # autopilot off
                if self.autopilot_flag:
                    print("autopilot off")
                    self.autopilot_flag = False
                    self.autopilot_ready = False
                    self.pos_con.disable()
            elif curr_rc["aux3"] == 2000:  # autopilot on
                if not self.autopilot_flag:
                    print("autopilot on")
                    self.autopilot_flag = True
                    self.pos_con.enable()
                else:
                    if self.pos_con.is_enabled():
                        rc_roll, rc_pitch, rc_throttle, rc_yaw = self.pos_con.update()
                        self.autopilot_ready = True

            if curr_rc["aux4"] == 1000:  # human control
                self.last_human_roll = curr_rc["roll"]
                self.last_human_pitch = curr_rc["pitch"]
                self.last_human_throttle = curr_rc["throttle"]
                self.last_human_yaw = curr_rc["yaw"]

            if self.autopilot_ready:
                # print(f"Autopilot control - roll: {rc_roll}")
                print(rc_roll, rc_pitch)
                self.msp.set_rc(
                    [
                        # self.last_human_roll,
                        rc_roll,
                        rc_pitch,
                        # rc_throttle,
                        self.last_human_throttle,
                        self.last_human_yaw,
                    ]
                )
            else:
                print(self.last_human_roll, self.last_human_pitch)
                # print(f"Human control - roll: {self.last_human_roll}")
                self.msp.set_rc(
                    [
                        self.last_human_roll,
                        self.last_human_pitch,
                        self.last_human_throttle,
                        self.last_human_yaw,
                    ]
                )
            time.sleep(self.dt - MSP_LATENCY)

    def close(self) -> None:
        self.msp.close()
