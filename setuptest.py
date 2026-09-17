#!/usr/bin/env python3
"""LD2410C + GY-85 bench test (standalone Raspberry Pi Python; no ROS/Docker).

CAUTION: LD2410C currently uses physical pins 8/10 (GPIO14/15, /dev/serial0).
Before --confirm-uart-exclusive, physically verify that GNSS TX is NOT also
connected to Pi pin 10, and no other device drives the same UART lines.
"""
from __future__ import annotations

import argparse
import math
import os
import threading
import time
from datetime import datetime


HEADER = bytes.fromhex('f4 f3 f2 f1')
FOOTER = bytes.fromhex('f8 f7 f6 f5')


def log(source: str, status: str, message: str) -> None:
    print(f'[{datetime.now().astimezone().isoformat(timespec="seconds")}] '
          f'[{source:<5}] [{status:<5}] {message}', flush=True)


def signed16(low: int, high: int) -> int:
    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value


class ImuReader:
    def __init__(self, bus_number: int = 1):
        self.bus_number = bus_number
        self.bus = None
        self.accel_ok = False
        self.gyro_ok = False

    def open(self) -> None:
        try:
            try:
                from smbus2 import SMBus
            except ImportError:
                from smbus import SMBus
        except ImportError as exc:
            raise RuntimeError('I2C Python library missing: sudo apt install python3-smbus') from exc
        self.bus = SMBus(self.bus_number)
        log('IMU', 'OPEN', f'/dev/i2c-{self.bus_number}')

        # Addresses 0x53, 0x68 and 0x2C were observed in an earlier i2cdetect.
        # 0x2C is still unidentified: do NOT probe or configure its registers.
        log('IMU', 'INFO', 'Previously observed addresses: 0x2C, 0x53, 0x68; 0x2C untouched')
        try:
            ident = self.bus.read_byte_data(0x53, 0x00)
            self.accel_ok = ident == 0xE5
            log('IMU', 'ID', f'0x53 register 0x00 = 0x{ident:02X}; '
                f'ADXL345 ID {"confirmed" if self.accel_ok else "NOT confirmed"}')
        except OSError as exc:
            log('IMU', 'ERROR', f'0x53 identity read failed: {exc}')

        try:
            ident = self.bus.read_byte_data(0x68, 0x00)
            # ITG-3200 family WHO_AM_I stores 7-bit I2C address in bits 1..6.
            self.gyro_ok = (ident & 0x7E) == 0x68
            log('IMU', 'ID', f'0x68 WHO_AM_I = 0x{ident:02X}; '
                f'ITG-3200-like ID {"consistent" if self.gyro_ok else "NOT confirmed"}')
        except OSError as exc:
            log('IMU', 'ERROR', f'0x68 identity read failed: {exc}')

        # Only configure chips that passed identification.
        if self.accel_ok:
            try:
                self.bus.write_byte_data(0x53, 0x31, 0x0B)  # FULL_RES, +/-16 g
                self.bus.write_byte_data(0x53, 0x2D, 0x08)  # measurement mode
                log('IMU', 'WRITE', '0x53: DATA_FORMAT=0x0B, POWER_CTL=0x08')
            except OSError as exc:
                self.accel_ok = False
                log('IMU', 'ERROR', f'0x53 setup failed: {exc}')
        if self.gyro_ok:
            try:
                self.bus.write_byte_data(0x68, 0x3E, 0x00)  # wake up
                self.bus.write_byte_data(0x68, 0x16, 0x18)  # FS_SEL=3 (2000 dps)
                log('IMU', 'WRITE', '0x68: PWR_MGM=0x00, DLPF_FS=0x18')
            except OSError as exc:
                self.gyro_ok = False
                log('IMU', 'ERROR', f'0x68 setup failed: {exc}')

    def sample(self) -> None:
        if self.accel_ok:
            try:
                b = self.bus.read_i2c_block_data(0x53, 0x32, 6)
                xyz = [signed16(b[i], b[i + 1]) * 0.0039 * 9.80665 for i in (0, 2, 4)]
                log('IMU', 'READ', 'accel [m/s^2]: x={:+.3f} y={:+.3f} z={:+.3f}'.format(*xyz))
            except OSError as exc:
                log('IMU', 'ERROR', f'accelerometer read failed: {exc}')
        if self.gyro_ok:
            try:
                b = self.bus.read_i2c_block_data(0x68, 0x1D, 6)
                xyz = [math.radians(signed16(b[i + 1], b[i]) / 14.375) for i in (0, 2, 4)]
                log('IMU', 'READ', 'gyro [rad/s]:   x={:+.4f} y={:+.4f} z={:+.4f}'.format(*xyz))
            except OSError as exc:
                log('IMU', 'ERROR', f'gyroscope read failed: {exc}')
        if not self.accel_ok and not self.gyro_ok:
            log('IMU', 'WAIT', 'Neither IMU chip passed ID/setup checks; no fake data printed')

    def close(self) -> None:
        if self.bus is not None:
            self.bus.close()
            self.bus = None


class RadarParser:
    def __init__(self):
        self.buf = bytearray()
        self.good_frames = 0
        self.bad_frames = 0

    def feed(self, chunk: bytes) -> list[dict]:
        self.buf.extend(chunk)
        reports = []
        while True:
            idx = self.buf.find(HEADER)
            if idx < 0:
                # Keep up to 3 trailing bytes in case the next read completes the header.
                self.buf = self.buf[-3:]
                break
            if idx:
                del self.buf[:idx]
            if len(self.buf) < 6:
                break
            length = int.from_bytes(self.buf[4:6], 'little')
            if not 11 <= length <= 128:
                self.bad_frames += 1
                del self.buf[0]
                continue
            n = 6 + length + 4
            if len(self.buf) < n:
                break
            frame = bytes(self.buf[:n])
            if frame[-4:] != FOOTER:
                self.bad_frames += 1
                del self.buf[0]
                continue
            del self.buf[:n]
            payload = frame[6:-4]
            # Basic target-data mode: 02 AA, 11-byte measurements, 55 00 trailer.
            if payload[:2] != b'\x02\xaa' or payload[-2:] != b'\x55\x00':
                self.bad_frames += 1
                continue
            state = payload[2]
            if state not in (0, 1, 2, 3):
                self.bad_frames += 1
                continue
            move = state in (1, 3)
            still = state in (2, 3)
            moving_cm = int.from_bytes(payload[3:5], 'little')
            stationary_cm = int.from_bytes(payload[6:8], 'little')
            detection_cm = int.from_bytes(payload[9:11], 'little')
            reports.append({
                'presence': move or still, 'moving': move, 'stationary': still,
                'moving_m': moving_cm / 100 if move else None,
                'stationary_m': stationary_cm / 100 if still else None,
                'detection_m': detection_cm / 100 if (move or still) else None,
                'moving_energy': payload[5], 'stationary_energy': payload[8],
            })
            self.good_frames += 1
        return reports


def imu_worker(args, stop: threading.Event):
    imu = ImuReader(args.imu_bus)
    try:
        imu.open()
        while not stop.is_set():
            imu.sample()
            stop.wait(max(0.1, args.imu_interval))
    except Exception as exc:
        log('IMU', 'ERROR', f'{type(exc).__name__}: {exc}')
    finally:
        imu.close()
        log('IMU', 'CLOSE', 'I2C closed')


def radar_worker(args, stop: threading.Event):
    try:
        import serial
    except ImportError:
        log('RADAR', 'ERROR', 'pyserial missing: sudo apt install python3-serial')
        return
    ser = None
    parser = RadarParser()
    received = 0
    last_status = time.monotonic()
    try:
        ser = serial.Serial(args.radar_port, args.radar_baud, timeout=0.3)
        log('RADAR', 'OPEN', f'{args.radar_port} -> {os.path.realpath(args.radar_port)}, {args.radar_baud} baud')
        while not stop.is_set():
            chunk = ser.read(128)
            received += len(chunk)
            for r in parser.feed(chunk):
                log('RADAR', 'READ', 'presence={presence} moving={moving} stationary={stationary} '
                    'moving_m={moving_m} stationary_m={stationary_m} detection_m={detection_m} '
                    'energy(m/s)={moving_energy}/{stationary_energy}'.format(**r))
            now = time.monotonic()
            if now - last_status >= 3:
                log('RADAR', 'INFO', f'bytes={received}, valid_frames={parser.good_frames}, '
                    f'invalid_frames={parser.bad_frames}')
                last_status = now
    except Exception as exc:
        log('RADAR', 'ERROR', f'{type(exc).__name__}: {exc}')
    finally:
        if ser is not None:
            ser.close()
        log('RADAR', 'CLOSE', 'serial closed')


def main() -> int:
    parser = argparse.ArgumentParser(description='GY-85 + LD2410C ONLY (no ROS, GNSS, or camera)')
    parser.add_argument('--duration', type=float, default=20.0, help='seconds; 0 means until Ctrl+C')
    parser.add_argument('--imu-bus', type=int, default=1)
    parser.add_argument('--imu-interval', type=float, default=1.0)
    parser.add_argument('--radar-port', default='/dev/serial0')
    parser.add_argument('--radar-baud', type=int, default=256000)
    parser.add_argument('--confirm-uart-exclusive', action='store_true',
                        help='I physically verified GNSS TX is NOT connected to Pi pin 10; '
                             'radar alone uses pins 8/10')
    args = parser.parse_args()
    log('SYSTEM', 'INFO', 'IMU I2C bus 1 + LD2410C UART; no ROS, GNSS, camera, or MCU used')
    if args.duration < 0:
        parser.error('--duration must be >= 0')
    if not args.confirm_uart_exclusive:
        parser.error('First physically confirm GNSS TX is disconnected from Pi pin 10 '
                     'and only radar uses UART pins 8/10, then rerun with '
                     '--confirm-uart-exclusive. UART settings/port listings do not prove this.')
    if not os.path.exists(args.radar_port):
        parser.error(f'UART device does not exist: {args.radar_port}')

    stop = threading.Event()
    tasks = [threading.Thread(target=imu_worker, args=(args, stop), name='imu'),
             threading.Thread(target=radar_worker, args=(args, stop), name='radar')]
    for task in tasks:
        task.start()
    try:
        if args.duration == 0:
            while any(t.is_alive() for t in tasks):
                time.sleep(0.1)
        else:
            stop.wait(args.duration)
    except KeyboardInterrupt:
        log('SYSTEM', 'STOP', 'Ctrl+C')
    finally:
        stop.set()
        for task in tasks:
            task.join(timeout=3)
        log('SYSTEM', 'DONE', 'test complete; verify readings change with physical movement')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
