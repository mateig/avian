import socket
import av
import cv2
import numpy as np

HOST = "192.168.4.69"
PORT = 8000
RAW_FILE = "dummy.h264"


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock, open(
        RAW_FILE, "wb"
    ) as raw_f:
        sock.connect((HOST, PORT))
        conn = sock.makefile("rb")

        decoder = av.CodecContext.create("h264", "r")

        while True:
            data = conn.read(4096)
            if not data:
                break

            raw_f.write(data)

            packets = decoder.parse(data)
            for packet in packets:
                frames = decoder.decode(packet)
                for frame in frames:
                    img = frame.to_ndarray(format="bgr24")
                    # roi = img[0:600, 250:1030]
                    cv2.imshow("Stream", img)
                    if cv2.waitKey(1) == 27:  # ESC
                        sock.close()
                        cv2.destroyAllWindows()
                        return


if __name__ == "__main__":
    main()
