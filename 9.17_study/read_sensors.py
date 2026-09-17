#!/usr/bin/env python3
"""
GY-85 (ADXL345 + ITG3205 + QMC5883P) + HLK-LD2410 실시간 데이터 읽기

설치: sudo apt install -y python3-smbus2 python3-serial
실행: python3 read_sensors.py            # IMU + 레이더
      python3 read_sensors.py --imu      # IMU만
      python3 read_sensors.py --radar    # 레이더만
종료: Ctrl+C   (권한 에러가 나면 sudo 붙여서 실행)
"""
import argparse
import math
import struct
import time

from smbus2 import SMBus
import serial

I2C_BUS = 1
RADAR_PORT = "/dev/serial0"
RADAR_BAUD = 256000


# ============================================================ ADXL345 (가속도)
class ADXL345:
    ADDR = 0x53
    SCALE = 0.0039  # g/LSB (FULL_RES 모드는 범위와 무관하게 약 3.9mg/LSB)

    def __init__(self, bus):
        self.bus = bus
        if bus.read_byte_data(self.ADDR, 0x00) != 0xE5:
            raise RuntimeError("ADXL345 DEVID 불일치")
        bus.write_byte_data(self.ADDR, 0x2C, 0x0A)  # BW_RATE: 100Hz
        bus.write_byte_data(self.ADDR, 0x31, 0x0B)  # DATA_FORMAT: FULL_RES, ±16g
        bus.write_byte_data(self.ADDR, 0x2D, 0x08)  # POWER_CTL: 측정 시작

    def read(self):
        raw = bytes(self.bus.read_i2c_block_data(self.ADDR, 0x32, 6))
        x, y, z = struct.unpack("<hhh", raw)  # 리틀 엔디안
        return x * self.SCALE, y * self.SCALE, z * self.SCALE


# ============================================================ ITG3205 (자이로)
class ITG3205:
    ADDR = 0x68
    SCALE = 14.375  # LSB/(°/s)

    def __init__(self, bus):
        self.bus = bus
        if (bus.read_byte_data(self.ADDR, 0x00) & 0x7E) != 0x68:
            raise RuntimeError("ITG3205 WHO_AM_I 불일치")
        bus.write_byte_data(self.ADDR, 0x3E, 0x80)  # PWR_MGM: 리셋
        time.sleep(0.05)
        bus.write_byte_data(self.ADDR, 0x3E, 0x01)  # 클럭 소스: X축 자이로 PLL
        bus.write_byte_data(self.ADDR, 0x15, 0x07)  # SMPLRT_DIV: 1kHz/(7+1) = 125Hz
        bus.write_byte_data(self.ADDR, 0x16, 0x1B)  # DLPF_FS: ±2000°/s, LPF 42Hz
        time.sleep(0.1)
        self.bias = (0.0, 0.0, 0.0)

    def _raw(self):
        raw = bytes(self.bus.read_i2c_block_data(self.ADDR, 0x1B, 8))
        return struct.unpack(">hhhh", raw)  # 온도, X, Y, Z (빅 엔디안)

    def calibrate(self, n=200):
        """정지 상태 평균값을 영점(bias)으로 사용"""
        sx = sy = sz = 0
        for _ in range(n):
            _, x, y, z = self._raw()
            sx, sy, sz = sx + x, sy + y, sz + z
            time.sleep(0.005)
        self.bias = (sx / n, sy / n, sz / n)

    def read(self):
        t, x, y, z = self._raw()
        bx, by, bz = self.bias
        gyro = ((x - bx) / self.SCALE, (y - by) / self.SCALE, (z - bz) / self.SCALE)
        temp_c = 35 + (t + 13200) / 280.0
        return gyro, temp_c


# ============================================================ QMC5883P (지자기)
class QMC5883P:
    ADDR = 0x2C
    SCALE = 3750.0  # LSB/Gauss (±8G 범위)

    def __init__(self, bus):
        self.bus = bus
        if bus.read_byte_data(self.ADDR, 0x00) != 0x80:
            raise RuntimeError("QMC5883P CHIP_ID 불일치")
        bus.write_byte_data(self.ADDR, 0x29, 0x06)  # 축 부호 정의 (데이터시트 권장 초기화)
        bus.write_byte_data(self.ADDR, 0x0B, 0x08)  # CTRL2: Set/Reset ON, 범위 ±8G
        bus.write_byte_data(self.ADDR, 0x0A, 0xCD)  # CTRL1: Normal 모드, ODR 200Hz
        time.sleep(0.05)

    def read(self):
        status = self.bus.read_byte_data(self.ADDR, 0x09)
        raw = bytes(self.bus.read_i2c_block_data(self.ADDR, 0x01, 6))
        x, y, z = struct.unpack("<hhh", raw)
        overflow = bool(status & 0x02)
        return (x / self.SCALE, y / self.SCALE, z / self.SCALE), overflow


# ============================================================ HLK-LD2410 (레이더)
class LD2410:
    HEAD = b"\xF4\xF3\xF2\xF1"
    TAIL = b"\xF8\xF7\xF6\xF5"
    STATES = {0: "타깃 없음", 1: "움직이는 타깃", 2: "정지 타깃", 3: "움직임+정지"}

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0)
        self.ser.reset_input_buffer()
        self.buf = bytearray()
        self.latest = None
        self.last_time = None
        self.frames = 0

    def poll(self):
        """UART에 쌓인 바이트를 모두 읽고 완성된 프레임을 해석"""
        self.buf += self.ser.read(self.ser.in_waiting or 1)
        while True:
            i = self.buf.find(self.HEAD)
            if i < 0:
                del self.buf[:-3]  # 헤더 일부가 끝에 걸쳐 있을 수 있어 3바이트 남김
                return
            del self.buf[:i]
            if len(self.buf) < 6:
                return
            length = struct.unpack_from("<H", self.buf, 4)[0]
            if length > 256:  # 비정상 길이 → 헤더 건너뛰고 다시 탐색
                del self.buf[:4]
                continue
            end = 6 + length + 4
            if len(self.buf) < end:
                return
            frame = bytes(self.buf[:end])
            del self.buf[:end]
            if frame[-4:] == self.TAIL:
                self._parse(frame[6:6 + length])

    def _parse(self, d):
        # d[0]: 데이터 종류(0x01 엔지니어링 / 0x02 기본), d[1]: 0xAA
        # 이어서 기본 타깃 정보 9바이트 (엔지니어링 모드도 앞부분은 동일)
        if len(d) < 11 or d[1] != 0xAA:
            return
        state, mv_dist, mv_energy, st_dist, st_energy, det_dist = struct.unpack_from("<BHBHBH", d, 2)
        self.latest = {
            "state": state,
            "moving_cm": mv_dist, "moving_energy": mv_energy,
            "static_cm": st_dist, "static_energy": st_energy,
            "detect_cm": det_dist,
        }
        self.last_time = time.time()
        self.frames += 1

    def close(self):
        self.ser.close()


# ============================================================ 메인
def try_init(name, fn):
    try:
        return fn()
    except Exception as e:
        print(f"[{name}] 초기화 실패: {e}")
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--imu", action="store_true", help="IMU만")
    ap.add_argument("--radar", action="store_true", help="레이더만")
    args = ap.parse_args()
    use_imu = args.imu or not args.radar
    use_radar = args.radar or not args.imu

    bus = SMBus(I2C_BUS) if use_imu else None
    acc = try_init("ADXL345", lambda: ADXL345(bus)) if use_imu else None
    gyro = try_init("ITG3205", lambda: ITG3205(bus)) if use_imu else None
    mag = try_init("QMC5883P", lambda: QMC5883P(bus)) if use_imu else None
    radar = try_init("LD2410", lambda: LD2410(RADAR_PORT, RADAR_BAUD)) if use_radar else None

    if gyro:
        print("자이로 영점 보정 중... 보드를 2초간 움직이지 마세요")
        gyro.calibrate()
    time.sleep(0.5)

    try:
        while True:
            lines = ["=== 센서 실시간 값 (Ctrl+C 종료) ===", ""]

            if acc:
                try:
                    ax, ay, az = acc.read()
                    norm = math.sqrt(ax * ax + ay * ay + az * az)
                    lines.append(f"가속도 [g]      x={ax:+6.2f}  y={ay:+6.2f}  z={az:+6.2f}   |a|={norm:.2f}")
                except OSError as e:
                    lines.append(f"가속도 읽기 오류: {e}")

            if gyro:
                try:
                    (gx, gy, gz), temp = gyro.read()
                    lines.append(f"각속도 [°/s]    x={gx:+7.1f} y={gy:+7.1f} z={gz:+7.1f}   온도={temp:.1f}°C")
                except OSError as e:
                    lines.append(f"자이로 읽기 오류: {e}")

            if mag:
                try:
                    (mx, my, mz), ovf = mag.read()
                    norm = math.sqrt(mx * mx + my * my + mz * mz)
                    heading = (math.degrees(math.atan2(my, mx)) + 360) % 360
                    lines.append(f"지자기 [Gauss]  x={mx:+6.3f}  y={my:+6.3f}  z={mz:+6.3f}   |B|={norm:.3f}"
                                 f"   방위(보정 전)={heading:5.1f}°" + ("  ⚠오버플로" if ovf else ""))
                except OSError as e:
                    lines.append(f"지자기 읽기 오류: {e}")

            if radar:
                radar.poll()
                r = radar.latest
                lines.append("")
                if r is None:
                    lines.append("레이더: 프레임 대기 중...")
                else:
                    age = time.time() - radar.last_time
                    lines.append(f"레이더 상태: {LD2410.STATES.get(r['state'], r['state'])}"
                                 f"   (프레임 {radar.frames}개, {age:.1f}초 전)")
                    lines.append(f"  움직이는 타깃  거리={r['moving_cm']:4d} cm  에너지={r['moving_energy']:3d}")
                    lines.append(f"  정지 타깃      거리={r['static_cm']:4d} cm  에너지={r['static_energy']:3d}")
                    lines.append(f"  감지 거리      {r['detect_cm']:4d} cm")

            print("\033[H\033[J" + "\n".join(lines), flush=True)  # 화면 지우고 다시 출력
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        if radar:
            radar.close()
        if bus:
            bus.close()


if __name__ == "__main__":
    main()