#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, Iterable, List, Optional, Tuple, Union


"""
Standalone non-ROS integrated sensor checker for the Ajou patrol robot.

This file is intentionally self-contained. It does not import this repository's
ROS2 packages, so it can be copied to Raspberry Pi OS and run with:

    python3 sensor_check.py

It never prints fake sensor success. Missing dependencies, missing devices,
invalid GNSS fixes, and unset LD2410C serial paths are reported explicitly.
"""


class SensorInitializationError(RuntimeError):
    pass


class SensorDependencyError(SensorInitializationError):
    pass


class SensorReadError(RuntimeError):
    pass


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


class Console:
    def __init__(self) -> None:
        self._lock = threading.Lock()

    def line(self, sensor: str, status: str, message: str) -> None:
        with self._lock:
            print("[{}] [{:<8}] [{:<6}] {}".format(utc_timestamp(), sensor, status, message), flush=True)


@dataclass(frozen=True)
class GnssFix:
    latitude: float
    longitude: float
    altitude: float = 0.0
    valid: bool = False
    status: int = -1
    service: int = 1
    hdop: Optional[float] = None
    sentence_type: str = ""


@dataclass
class GnssReadResult:
    timestamp: str
    raw_line: str
    fix: Optional[GnssFix]
    nmea_received: bool
    valid_fix: bool


def nmea_checksum(sentence: str) -> int:
    body = _body_without_checksum(sentence)
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return checksum


def validate_checksum(sentence: str) -> bool:
    sentence = sentence.strip()
    if "*" not in sentence:
        return False
    try:
        supplied = int(sentence.rsplit("*", 1)[1][:2], 16)
    except ValueError:
        return False
    return nmea_checksum(sentence) == supplied


def parse_lat_lon(raw_value: str, hemisphere: str) -> Optional[float]:
    if not raw_value or not hemisphere:
        return None
    try:
        dot_index = raw_value.index(".")
        degree_digits = dot_index - 2
        degrees = float(raw_value[:degree_digits])
        minutes = float(raw_value[degree_digits:])
    except (ValueError, IndexError):
        return None

    decimal = degrees + minutes / 60.0
    if hemisphere.upper() in ("S", "W"):
        decimal *= -1.0
    elif hemisphere.upper() not in ("N", "E"):
        return None
    return decimal


def parse_nmea_sentence(sentence: str, require_checksum: bool = True) -> Optional[GnssFix]:
    sentence = sentence.strip()
    if not sentence.startswith("$"):
        return None
    if require_checksum and not validate_checksum(sentence):
        return None

    fields = _body_without_checksum(sentence).split(",")
    if not fields:
        return None

    sentence_type = fields[0].upper()[-3:]
    if sentence_type == "GGA":
        return _parse_gga(fields)
    if sentence_type == "RMC":
        return _parse_rmc(fields)
    return None


def _parse_gga(fields: List[str]) -> Optional[GnssFix]:
    if len(fields) < 10:
        return None
    lat = parse_lat_lon(fields[2], fields[3])
    lon = parse_lat_lon(fields[4], fields[5])
    if lat is None or lon is None:
        return None
    try:
        fix_quality = int(fields[6] or "0")
    except ValueError:
        fix_quality = 0
    try:
        altitude = float(fields[9] or "0.0")
    except ValueError:
        altitude = 0.0
    try:
        hdop = float(fields[8]) if fields[8] else None
    except ValueError:
        hdop = None

    return GnssFix(
        latitude=lat,
        longitude=lon,
        altitude=altitude,
        valid=fix_quality > 0,
        status=0 if fix_quality > 0 else -1,
        hdop=hdop,
        sentence_type="GGA",
    )


def _parse_rmc(fields: List[str]) -> Optional[GnssFix]:
    if len(fields) < 7:
        return None
    lat = parse_lat_lon(fields[3], fields[4])
    lon = parse_lat_lon(fields[5], fields[6])
    if lat is None or lon is None:
        return None
    valid = fields[2].upper() == "A"
    return GnssFix(
        latitude=lat,
        longitude=lon,
        altitude=0.0,
        valid=valid,
        status=0 if valid else -1,
        sentence_type="RMC",
    )


def _body_without_checksum(sentence: str) -> str:
    sentence = sentence.strip()
    if sentence.startswith("$"):
        sentence = sentence[1:]
    return sentence.split("*", 1)[0]


class GnssSerialReader:
    def __init__(
        self,
        port: str = "/dev/serial0",
        baud: int = 9600,
        timeout_s: float = 0.5,
        require_checksum: bool = True,
    ) -> None:
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.require_checksum = require_checksum
        self.serial = None

    def open(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise SensorDependencyError("install python3-serial") from exc
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.timeout_s)
        except Exception as exc:
            raise SensorInitializationError(
                "failed to open GNSS serial port {} at {}: {}".format(self.port, self.baud, exc)
            ) from exc

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()
        self.serial = None

    def read_once(self) -> Optional[GnssReadResult]:
        if self.serial is None:
            raise SensorInitializationError("GNSS serial port is not open")
        try:
            raw = self.serial.readline()
        except Exception as exc:
            raise SensorReadError("GNSS read failed: {}".format(exc)) from exc
        if not raw:
            return None
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return None
        fix = parse_nmea_sentence(line, require_checksum=self.require_checksum)
        return GnssReadResult(
            timestamp=utc_timestamp(),
            raw_line=line,
            fix=fix,
            nmea_received=line.startswith("$"),
            valid_fix=bool(fix and fix.valid),
        )


DATA_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])
DATA_FOOTER = bytes([0xF8, 0xF7, 0xF6, 0xF5])
MAX_FRAME_LENGTH = 128


@dataclass(frozen=True)
class RadarReport:
    target_state: int
    presence: bool
    moving_target: bool
    stationary_target: bool
    moving_distance_m: Optional[float]
    stationary_distance_m: Optional[float]
    detection_distance_m: Optional[float]
    moving_energy: Optional[int] = None
    stationary_energy: Optional[int] = None


@dataclass
class RadarReadResult:
    timestamp: str
    bytes_received: int
    reports: List[RadarReport]


class LD2410CParser:
    def __init__(self, max_frame_length: int = MAX_FRAME_LENGTH) -> None:
        self._buffer = bytearray()
        self._max_frame_length = max_frame_length

    def feed(self, data: Union[bytes, bytearray, Iterable[int]]) -> List[RadarReport]:
        self._buffer.extend(bytes(data))
        reports: List[RadarReport] = []

        while True:
            start = self._buffer.find(DATA_HEADER)
            if start < 0:
                self._keep_possible_header_prefix()
                break
            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < 10:
                break

            payload_len = int.from_bytes(self._buffer[4:6], "little")
            if payload_len <= 0 or payload_len > self._max_frame_length:
                del self._buffer[0]
                continue

            frame_len = 4 + 2 + payload_len + 4
            if len(self._buffer) < frame_len:
                break
            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            if frame[-4:] != DATA_FOOTER:
                continue

            report = parse_radar_data_payload(frame[6:-4])
            if report is not None:
                reports.append(report)

        return reports

    def _keep_possible_header_prefix(self) -> None:
        for keep in range(min(len(DATA_HEADER) - 1, len(self._buffer)), 0, -1):
            if DATA_HEADER.startswith(bytes(self._buffer[-keep:])):
                del self._buffer[:-keep]
                return
        self._buffer.clear()


def parse_radar_data_payload(payload: bytes) -> Optional[RadarReport]:
    if len(payload) < 11:
        return None
    if payload[0] != 0x02 or payload[1] != 0xAA:
        return None

    target_state = payload[2]
    moving_cm = int.from_bytes(payload[3:5], "little")
    moving_energy = payload[5]
    stationary_cm = int.from_bytes(payload[6:8], "little")
    stationary_energy = payload[8]
    detection_cm = int.from_bytes(payload[9:11], "little")

    moving_target = target_state in (1, 3)
    stationary_target = target_state in (2, 3)
    presence = moving_target or stationary_target

    return RadarReport(
        target_state=target_state,
        presence=presence,
        moving_target=moving_target,
        stationary_target=stationary_target,
        moving_distance_m=(moving_cm / 100.0) if moving_target else None,
        stationary_distance_m=(stationary_cm / 100.0) if stationary_target else None,
        detection_distance_m=(detection_cm / 100.0) if presence else None,
        moving_energy=moving_energy,
        stationary_energy=stationary_energy,
    )


class LD2410CSerialReader:
    def __init__(self, port: str, baud: int = 256000, timeout_s: float = 0.2) -> None:
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.serial = None
        self.parser = LD2410CParser()

    def open(self) -> None:
        if not self.port:
            raise SensorInitializationError("LD2410C serial port is not configured")
        if self.port == "/dev/serial0":
            raise SensorInitializationError("refusing /dev/serial0 for LD2410C because GNSS uses it")
        try:
            import serial
        except ImportError as exc:
            raise SensorDependencyError("install python3-serial") from exc
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.timeout_s)
        except Exception as exc:
            raise SensorInitializationError(
                "failed to open LD2410C serial port {} at {}: {}".format(self.port, self.baud, exc)
            ) from exc

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()
        self.serial = None

    def read_once(self, size: int = 64) -> RadarReadResult:
        if self.serial is None:
            raise SensorInitializationError("LD2410C serial port is not open")
        try:
            data = self.serial.read(size)
        except Exception as exc:
            raise SensorReadError("LD2410C read failed: {}".format(exc)) from exc
        reports = self.parser.feed(data) if data else []
        return RadarReadResult(timestamp=utc_timestamp(), bytes_received=len(data), reports=reports)


OBSERVED_ADDRESSES = (0x2C, 0x53, 0x68)
ADXL345_DEVID_REGISTER = 0x00
ADXL345_EXPECTED_DEVID = 0xE5
ADXL345_POWER_CTL = 0x2D
ADXL345_DATA_FORMAT = 0x31
ADXL345_DATAX0 = 0x32
ITG_WHO_AM_I = 0x00
ITG_DLPF_FS = 0x16
ITG_GYRO_XOUT_H = 0x1D
ITG_PWR_MGM = 0x3E


@dataclass
class I2CProbeResult:
    address: int
    present: bool
    probe_value: Optional[int] = None
    error: Optional[str] = None


@dataclass
class Gy85ProbeSummary:
    bus_number: int
    checked_at: str
    address_results: Dict[int, I2CProbeResult] = field(default_factory=dict)
    accelerometer_verified: bool = False
    gyro_verified: bool = False
    accel_device_id: Optional[int] = None
    gyro_who_am_i: Optional[int] = None
    third_device_value: Optional[int] = None
    errors: List[str] = field(default_factory=list)


@dataclass
class Gy85Sample:
    timestamp: str
    acceleration_mps2: Optional[Tuple[float, float, float]] = None
    angular_velocity_radps: Optional[Tuple[float, float, float]] = None


class Gy85Interface:
    def __init__(
        self,
        bus_number: int = 1,
        accelerometer_address: int = 0x53,
        gyro_address: int = 0x68,
        third_device_address: int = 0x2C,
    ) -> None:
        self.bus_number = bus_number
        self.accelerometer_address = accelerometer_address
        self.gyro_address = gyro_address
        self.third_device_address = third_device_address
        self.bus = None
        self.accelerometer_verified = False
        self.gyro_verified = False

    def open(self) -> None:
        self.bus = open_smbus(self.bus_number)

    def close(self) -> None:
        if self.bus is not None and hasattr(self.bus, "close"):
            self.bus.close()
        self.bus = None

    def probe(self) -> Gy85ProbeSummary:
        self._require_bus()
        summary = Gy85ProbeSummary(bus_number=self.bus_number, checked_at=utc_timestamp())
        for address in OBSERVED_ADDRESSES:
            summary.address_results[address] = probe_address(self.bus, address)

        accel = summary.address_results.get(self.accelerometer_address)
        if accel is not None and accel.present:
            try:
                device_id = self.bus.read_byte_data(self.accelerometer_address, ADXL345_DEVID_REGISTER)
                summary.accel_device_id = device_id
                summary.accelerometer_verified = device_id == ADXL345_EXPECTED_DEVID
                self.accelerometer_verified = summary.accelerometer_verified
            except Exception as exc:
                summary.errors.append("accelerometer identity read failed: {}".format(exc))

        gyro = summary.address_results.get(self.gyro_address)
        if gyro is not None and gyro.present:
            try:
                who_am_i = self.bus.read_byte_data(self.gyro_address, ITG_WHO_AM_I)
                summary.gyro_who_am_i = who_am_i
                summary.gyro_verified = (who_am_i & 0x7E) == self.gyro_address
                self.gyro_verified = summary.gyro_verified
            except Exception as exc:
                summary.errors.append("gyro identity read failed: {}".format(exc))

        third = summary.address_results.get(self.third_device_address)
        if third is not None and third.present:
            summary.third_device_value = third.probe_value

        return summary

    def initialize_verified_devices(self) -> None:
        self._require_bus()
        try:
            if self.accelerometer_verified:
                self.bus.write_byte_data(self.accelerometer_address, ADXL345_POWER_CTL, 0x08)
                self.bus.write_byte_data(self.accelerometer_address, ADXL345_DATA_FORMAT, 0x0B)
            if self.gyro_verified:
                self.bus.write_byte_data(self.gyro_address, ITG_PWR_MGM, 0x00)
                self.bus.write_byte_data(self.gyro_address, ITG_DLPF_FS, 0x18)
        except Exception as exc:
            raise SensorInitializationError("GY-85 verified-device initialization failed: {}".format(exc)) from exc

    def read_acceleration(self) -> Tuple[float, float, float]:
        self._require_bus()
        if not self.accelerometer_verified:
            raise SensorReadError("accelerometer identity is not verified")
        data = self.bus.read_i2c_block_data(self.accelerometer_address, ADXL345_DATAX0, 6)
        counts = [_to_signed_16(data[i], data[i + 1]) for i in (0, 2, 4)]
        scale = 0.0039 * 9.80665
        return tuple(value * scale for value in counts)  # type: ignore[return-value]

    def read_gyro(self) -> Tuple[float, float, float]:
        self._require_bus()
        if not self.gyro_verified:
            raise SensorReadError("gyro identity is not verified")
        data = self.bus.read_i2c_block_data(self.gyro_address, ITG_GYRO_XOUT_H, 6)
        counts = [_to_signed_16(data[i + 1], data[i]) for i in (0, 2, 4)]
        deg_s = [value / 14.375 for value in counts]
        return tuple(math.radians(value) for value in deg_s)  # type: ignore[return-value]

    def read_sample(self) -> Gy85Sample:
        acceleration = None
        angular_velocity = None
        if self.accelerometer_verified:
            acceleration = self.read_acceleration()
        if self.gyro_verified:
            angular_velocity = self.read_gyro()
        return Gy85Sample(
            timestamp=utc_timestamp(),
            acceleration_mps2=acceleration,
            angular_velocity_radps=angular_velocity,
        )

    def _require_bus(self) -> None:
        if self.bus is None:
            raise SensorInitializationError("I2C bus is not open")


def open_smbus(bus_number: int):
    try:
        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus
    except ImportError as exc:
        raise SensorDependencyError("install python3-smbus or smbus2") from exc
    try:
        return SMBus(bus_number)
    except Exception as exc:
        raise SensorInitializationError("failed to open I2C bus {}: {}".format(bus_number, exc)) from exc


def probe_address(bus, address: int) -> I2CProbeResult:
    try:
        if hasattr(bus, "write_quick"):
            bus.write_quick(address)
            return I2CProbeResult(address=address, present=True)
    except Exception:
        pass
    try:
        value = bus.read_byte(address)
        return I2CProbeResult(address=address, present=True, probe_value=value)
    except Exception as exc:
        return I2CProbeResult(address=address, present=False, error=str(exc))


def _to_signed_16(lo: int, hi: int) -> int:
    value = (hi << 8) | lo
    if value & 0x8000:
        value -= 0x10000
    return value


@dataclass
class CameraCaptureResult:
    timestamp: str
    backend: str
    path: str
    width: Optional[int] = None
    height: Optional[int] = None
    size_bytes: Optional[int] = None


class Picamera2StreamBackend:
    encoding = "rgb8"

    def __init__(self, width: int = 640, height: int = 480) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise SensorDependencyError("install python3-picamera2") from exc
        self.width = width
        self.height = height
        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        self.camera.configure(config)
        self.camera.start()
        time.sleep(0.3)

    def capture_file(self, path: str) -> CameraCaptureResult:
        self.camera.capture_file(path)
        return _camera_result("picamera2", path, self.width, self.height)

    def close(self) -> None:
        self.camera.stop()


class OpenCvStreamBackend:
    def __init__(self, device_index: int = 0, width: int = 640, height: int = 480) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise SensorDependencyError("install python3-opencv") from exc
        self.cv2 = cv2
        self.cap = cv2.VideoCapture(device_index)
        if not self.cap.isOpened():
            raise SensorInitializationError("OpenCV camera device {} did not open".format(device_index))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def save(self, frame, path: str) -> bool:
        return bool(self.cv2.imwrite(path, frame))

    def close(self) -> None:
        self.cap.release()


class CameraChecker:
    def __init__(
        self,
        backend: str = "auto",
        output_dir: str = "sensor_check_captures",
        width: int = 640,
        height: int = 480,
        device_index: int = 0,
    ) -> None:
        self.backend = backend
        self.output_dir = output_dir
        self.width = width
        self.height = height
        self.device_index = device_index
        self._stream_backend = None

    def close(self) -> None:
        if self._stream_backend is not None and hasattr(self._stream_backend, "close"):
            self._stream_backend.close()
        self._stream_backend = None

    def capture_once(self) -> CameraCaptureResult:
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, "camera_check_{}.jpg".format(_filename_timestamp()))
        errors: List[str] = []
        candidates = [self.backend] if self.backend != "auto" else ["picamera2", "rpicam-still", "opencv"]
        for candidate in candidates:
            try:
                if candidate == "picamera2":
                    self._stream_backend = Picamera2StreamBackend(self.width, self.height)
                    return self._stream_backend.capture_file(path)
                if candidate == "rpicam-still":
                    return capture_with_rpicam_still(path, self.width, self.height)
                if candidate == "opencv":
                    self._stream_backend = OpenCvStreamBackend(self.device_index, self.width, self.height)
                    frame = self._stream_backend.read()
                    if frame is None:
                        raise SensorReadError("OpenCV returned no frame")
                    if not self._stream_backend.save(frame, path):
                        raise SensorReadError("OpenCV failed to save captured frame")
                    h, w = frame.shape[:2]
                    return _camera_result("opencv", path, w, h)
                errors.append("{}: unsupported backend".format(candidate))
            except Exception as exc:
                errors.append("{}: {}".format(candidate, exc))
                self.close()
        raise SensorInitializationError("; ".join(errors))


def capture_with_rpicam_still(path: str, width: int, height: int) -> CameraCaptureResult:
    command = shutil.which("rpicam-still") or shutil.which("libcamera-still")
    if command is None:
        raise SensorDependencyError("rpicam-still/libcamera-still command not found")
    args = [command, "-n", "-t", "1000", "--width", str(width), "--height", str(height), "-o", path]
    completed = subprocess.run(args, capture_output=True, text=True, timeout=10.0, check=False)
    if completed.returncode != 0:
        stderr = completed.stderr.strip() or completed.stdout.strip()
        raise SensorReadError("{} failed: {}".format(os.path.basename(command), stderr))
    return _camera_result(os.path.basename(command), path, width, height)


def _filename_timestamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _camera_result(backend: str, path: str, width: Optional[int], height: Optional[int]) -> CameraCaptureResult:
    return CameraCaptureResult(
        timestamp=utc_timestamp(),
        backend=backend,
        path=path,
        width=width,
        height=height,
        size_bytes=os.path.getsize(path) if os.path.exists(path) else None,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a standalone non-ROS integrated hardware communication check for patrol robot sensors."
    )
    parser.add_argument("--duration", type=float, default=20.0, help="seconds to run streaming checks; 0 means until Ctrl+C")
    parser.add_argument("--imu-bus", type=int, default=1)
    parser.add_argument("--imu-interval", type=float, default=1.0)
    parser.add_argument("--gnss-port", default="/dev/serial0")
    parser.add_argument("--gnss-baud", type=int, default=9600)
    parser.add_argument("--gnss-timeout", type=float, default=0.5)
    parser.add_argument("--radar-port", default=os.environ.get("LD2410C_PORT", ""))
    parser.add_argument("--radar-baud", type=int, default=256000)
    parser.add_argument("--radar-timeout", type=float, default=0.2)
    parser.add_argument("--camera-backend", default="auto", choices=["auto", "picamera2", "rpicam-still", "opencv"])
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-dir", default="sensor_check_captures")
    return parser.parse_args()


def run_imu(args: argparse.Namespace, stop_event: threading.Event, console: Console) -> None:
    sensor = "IMU"
    imu = Gy85Interface(bus_number=args.imu_bus)
    try:
        imu.open()
        console.line(sensor, "OK", "I2C bus {} opened".format(args.imu_bus))
        summary = imu.probe()
        for address in OBSERVED_ADDRESSES:
            result = summary.address_results[address]
            if result.present:
                detail = "probe_value=0x{:02X}".format(result.probe_value) if result.probe_value is not None else "ack"
                console.line(sensor, "OK", "required address 0x{:02X} present ({})".format(address, detail))
            else:
                console.line(sensor, "ERROR", "required address 0x{:02X} missing: {}".format(address, result.error))

        if summary.accelerometer_verified:
            console.line(sensor, "OK", "accelerometer verified at 0x53, device_id=0x{:02X}".format(summary.accel_device_id))
        else:
            value = "none" if summary.accel_device_id is None else "0x{:02X}".format(summary.accel_device_id)
            console.line(sensor, "WARN", "accelerometer not verified at 0x53, device_id={}; accel values skipped".format(value))

        if summary.gyro_verified:
            console.line(sensor, "OK", "gyro verified at 0x68, who_am_i=0x{:02X}".format(summary.gyro_who_am_i))
        else:
            value = "none" if summary.gyro_who_am_i is None else "0x{:02X}".format(summary.gyro_who_am_i)
            console.line(sensor, "WARN", "gyro not verified at 0x68, who_am_i={}; gyro values skipped".format(value))

        if summary.address_results[0x2C].present:
            console.line(sensor, "INFO", "0x2C responds, but chip identity is unverified and not used as magnetometer")
        for error in summary.errors:
            console.line(sensor, "WARN", error)

        imu.initialize_verified_devices()
        while not stop_event.is_set():
            try:
                sample = imu.read_sample()
                parts: List[str] = []
                if sample.acceleration_mps2 is not None:
                    ax, ay, az = sample.acceleration_mps2
                    parts.append("accel_mps2=({:+.3f}, {:+.3f}, {:+.3f})".format(ax, ay, az))
                if sample.angular_velocity_radps is not None:
                    gx, gy, gz = sample.angular_velocity_radps
                    parts.append("gyro_radps=({:+.4f}, {:+.4f}, {:+.4f})".format(gx, gy, gz))
                if parts:
                    console.line(sensor, "DATA", "{} {}".format(sample.timestamp, " ".join(parts)))
                else:
                    console.line(sensor, "WARN", "no verified accelerometer/gyro measurement available")
            except SensorReadError as exc:
                console.line(sensor, "ERROR", str(exc))
            stop_event.wait(max(args.imu_interval, 0.1))
    except Exception as exc:
        console.line(sensor, "ERROR", str(exc))
    finally:
        imu.close()
        console.line(sensor, "CLOSE", "I2C bus closed")


def run_gnss(args: argparse.Namespace, stop_event: threading.Event, console: Console) -> None:
    sensor = "GNSS"
    reader = GnssSerialReader(port=args.gnss_port, baud=args.gnss_baud, timeout_s=args.gnss_timeout)
    last_wait = 0.0
    try:
        reader.open()
        console.line(sensor, "OK", "serial opened: {} at {} baud".format(args.gnss_port, args.gnss_baud))
        while not stop_event.is_set():
            try:
                result = reader.read_once()
            except SensorReadError as exc:
                console.line(sensor, "ERROR", str(exc))
                stop_event.wait(1.0)
                continue
            if result is None:
                now = time.monotonic()
                if now - last_wait >= 3.0:
                    console.line(sensor, "WAIT", "no NMEA line received yet")
                    last_wait = now
                continue
            if result.fix is None:
                console.line(sensor, "DATA", "NMEA/raw received but no parsed fix: {}".format(result.raw_line[:100]))
                continue
            fix = result.fix
            if result.valid_fix:
                console.line(
                    sensor,
                    "OK",
                    "{} valid {} fix lat={:.8f} lon={:.8f} alt={:.2f}".format(
                        result.timestamp,
                        fix.sentence_type,
                        fix.latitude,
                        fix.longitude,
                        fix.altitude,
                    ),
                )
            else:
                console.line(
                    sensor,
                    "DATA",
                    "{} NMEA received, parsed {}, but position fix is invalid/void".format(result.timestamp, fix.sentence_type),
                )
    except Exception as exc:
        console.line(sensor, "ERROR", str(exc))
    finally:
        reader.close()
        console.line(sensor, "CLOSE", "serial port closed")


def run_camera(args: argparse.Namespace, stop_event: threading.Event, console: Console) -> None:
    sensor = "CAMERA"
    checker = CameraChecker(
        backend=args.camera_backend,
        output_dir=args.camera_dir,
        width=args.camera_width,
        height=args.camera_height,
    )
    try:
        console.line(sensor, "INIT", "capture backend={} size={}x{}".format(args.camera_backend, args.camera_width, args.camera_height))
        result = checker.capture_once()
        console.line(
            sensor,
            "OK",
            "{} captured via {}: {} size_bytes={}".format(result.timestamp, result.backend, result.path, result.size_bytes),
        )
    except Exception as exc:
        console.line(sensor, "ERROR", str(exc))
    finally:
        checker.close()
        console.line(sensor, "CLOSE", "camera closed")


def run_radar(args: argparse.Namespace, stop_event: threading.Event, console: Console) -> None:
    sensor = "RADAR"
    if not args.radar_port:
        console.line(
            sensor,
            "WAIT",
            "LD2410C port is not configured; set --radar-port or LD2410C_PORT after discovering a separate UART",
        )
        return
    reader = LD2410CSerialReader(port=args.radar_port, baud=args.radar_baud, timeout_s=args.radar_timeout)
    last_wait = 0.0
    bytes_seen = 0
    try:
        reader.open()
        console.line(sensor, "OK", "serial opened: {} at {} baud".format(args.radar_port, args.radar_baud))
        while not stop_event.is_set():
            try:
                result = reader.read_once()
            except SensorReadError as exc:
                console.line(sensor, "ERROR", str(exc))
                stop_event.wait(1.0)
                continue
            bytes_seen += result.bytes_received
            if result.reports:
                for report in result.reports:
                    console.line(
                        sensor,
                        "DATA",
                        (
                            "{} presence={} moving={} stationary={} moving_m={} "
                            "stationary_m={} detection_m={}"
                        ).format(
                            result.timestamp,
                            report.presence,
                            report.moving_target,
                            report.stationary_target,
                            report.moving_distance_m,
                            report.stationary_distance_m,
                            report.detection_distance_m,
                        ),
                    )
            else:
                now = time.monotonic()
                if now - last_wait >= 3.0:
                    console.line(sensor, "WAIT", "bytes_received_total={}, waiting for valid LD2410C frame".format(bytes_seen))
                    last_wait = now
    except Exception as exc:
        console.line(sensor, "ERROR", str(exc))
    finally:
        reader.close()
        console.line(sensor, "CLOSE", "serial port closed")


def main() -> int:
    args = parse_args()
    console = Console()
    stop_event = threading.Event()
    console.line("SYSTEM", "START", "standalone non-ROS integrated sensor check started")
    console.line("SYSTEM", "INFO", "duration={}s; Ctrl+C stops all checks and closes devices".format(args.duration))

    workers = [
        threading.Thread(target=run_imu, args=(args, stop_event, console), name="imu", daemon=True),
        threading.Thread(target=run_gnss, args=(args, stop_event, console), name="gnss", daemon=True),
        threading.Thread(target=run_camera, args=(args, stop_event, console), name="camera", daemon=True),
        threading.Thread(target=run_radar, args=(args, stop_event, console), name="radar", daemon=True),
    ]
    for worker in workers:
        worker.start()

    try:
        if args.duration <= 0:
            while any(worker.is_alive() for worker in workers):
                time.sleep(0.2)
        else:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline and any(worker.is_alive() for worker in workers):
                time.sleep(0.2)
    except KeyboardInterrupt:
        console.line("SYSTEM", "STOP", "Ctrl+C received")
    finally:
        stop_event.set()
        for worker in workers:
            worker.join(timeout=3.0)
        console.line("SYSTEM", "DONE", "sensor check finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
