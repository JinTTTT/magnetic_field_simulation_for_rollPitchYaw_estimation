"""Xsens orientation decoding and live acquisition."""

from collections import deque
import math
import statistics
import threading
import time

from . import xsens


def wrap180(angle):
    return (angle + 180.0) % 360.0 - 180.0


def circular_mean_deg(values):
    sine = statistics.fmean(math.sin(math.radians(value)) for value in values)
    cosine = statistics.fmean(math.cos(math.radians(value)) for value in values)
    return math.degrees(math.atan2(sine, cosine))


def yaw_stddev_deg(values, mean):
    differences = [wrap180(value - mean) for value in values]
    return statistics.stdev(differences) if len(differences) > 1 else 0.0


def orientation_from_payload(payload):
    data = xsens.parse_mtdata2(payload)
    quaternion = data.get("Quaternion")
    if not isinstance(quaternion, list) and isinstance(data.get("EulerAngles"), list):
        quaternion = xsens.euler_to_quat(*data["EulerAngles"])
    if not isinstance(quaternion, list) and not isinstance(quaternion, tuple):
        return None

    quaternion = xsens.quat_mult(quaternion, xsens.MOUNT_QUAT)
    quaternion = xsens.quat_mult(
        xsens.AXIS_FIX,
        xsens.quat_mult(quaternion, xsens.AXIS_FIX),
    )
    roll, pitch, yaw = xsens.quat_to_rpy(quaternion)
    return yaw, pitch, roll


class LiveIMU:
    """Continuously retain timestamped Xsens yaw, pitch, and roll samples."""

    def __init__(self, serial_port, max_samples=10000):
        self.serial_port = serial_port
        self.samples = deque(maxlen=max_samples)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._read, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def _read(self):
        try:
            for message_id, payload in xsens.frames(self.serial_port):
                if self.stop_event.is_set():
                    break
                if message_id != xsens.MID_MTDATA2:
                    continue
                orientation = orientation_from_payload(payload)
                if orientation is not None:
                    with self.lock:
                        self.samples.append((time.monotonic(), *orientation))
        except Exception as error:
            if not self.stop_event.is_set():
                self.error = error

    def latest(self):
        with self.lock:
            return None if not self.samples else self.samples[-1][1:]

    def wait_for_sample(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.error is not None:
                raise RuntimeError("IMU reader stopped") from self.error
            sample = self.latest()
            if sample is not None:
                return sample
            time.sleep(0.02)
        raise TimeoutError("no orientation frame received from the IMU")

    def samples_between(self, start, end):
        with self.lock:
            return [sample[1:] for sample in self.samples if start <= sample[0] <= end]
