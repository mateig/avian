# drone/msp.py
import struct

import serial


class MSP:
    def __init__(self, port="/dev/serial0", baudrate=115200, timeout=0.1) -> None:
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def _read_bytes(self, n: int) -> bytes:
        b = self.ser.read(n)
        if len(b) != n:
            raise TimeoutError()
        return b

    def _checksum(self, payload: bytes) -> int:
        checksum = 0
        for byte in payload:
            checksum ^= byte
        return checksum

    def _request(self, cmd: int, data: bytes = b"") -> bytes:
        self.ser.reset_input_buffer()

        # send
        size = len(data)
        payload = bytes([size, cmd]) + data
        checksum = self._checksum(payload)
        frame = b"$M<" + payload + bytes([checksum])
        self.ser.write(frame)

        # read
        rb = self._read_bytes

        header = rb(3)
        if header == b"$M!":
            raise ValueError("MSP returned error")
        elif header != b"$M>":
            raise ValueError("Invalid MSP header")

        resp_size = rb(1)[0]
        resp_cmd = rb(1)[0]
        resp_payload = rb(resp_size)
        resp_checksum = rb(1)[0]

        if resp_checksum != self._checksum(bytes([resp_size, resp_cmd]) + resp_payload):
            raise ValueError("Invalid MSP checksum")

        if resp_cmd != cmd:
            raise ValueError("Response command does not match request")

        return resp_payload

    def close(self) -> None:
        if hasattr(self, "ser") and self.ser is not None and self.ser.is_open:
            self.ser.close()

    def get_raw_imu(self) -> dict:
        p = self._request(102)
        accx, accy, accz, gyrx, gyry, gyrz, magx, magy, magz = struct.unpack("<9h", p)
        return {
            "accx": accx,
            "accy": accy,
            "accz": accz,
            "gyrx": gyrx,
            "gyry": gyry,
            "gyrz": gyrz,
            "magx": magx,
            "magy": magy,
            "magz": magz,
        }

    def get_rc(self) -> dict:
        p = self._request(105)
        count = len(p) // 2
        channels = struct.unpack("<" + "H" * count, p)
        names = [
            "roll",
            "pitch",
            "yaw",
            "throttle",
            "aux1",
            "aux2",
            "aux3",
            "aux4",
            "aux5",
            "aux6",
            "aux7",
            "aux8",
            "aux9",
            "aux10",
            "aux11",
            "aux12",
        ][:count]
        return dict(zip(names, channels))

    def get_attitude(self) -> dict:
        p = self._request(108)
        angx, angy, heading = struct.unpack("<hhh", p)
        return {
            "roll": angx / 10.0,
            "pitch": angy / 10.0,
            "yaw": heading,
        }

    def get_altitude(self) -> dict:
        p = self._request(109)
        est_alt, vario = struct.unpack("<i h", p)  # vario doesn't work
        return {"altitude": est_alt}

    def get_analog(self) -> dict:
        p = self._request(110)
        vbat = p[0]
        int_power, rssi, amperage = struct.unpack("<3H", p[1:7])

        return {
            "voltage": vbat / 10.0,
            "power_sum": int_power,
            "rssi": rssi,
            "amperage": amperage,
        }

    def set_rc(self, channels: list) -> None:
        # roll, pitch, throttle, yaw
        if len(channels) != 4:
            raise ValueError("Expected 4 channels for roll, pitch, throttle, yaw")
        payload = struct.pack("<4H", *channels)
        self._request(200, payload)
