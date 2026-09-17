import serial, sys, time

PORT = "/dev/serial0"     # verify with: ls /dev/tty*
BAUD = 256000             # LD2410 default — unusual value

HDR, TAIL = b"\xF4\xF3\xF2\xF1", b"\xF8\xF7\xF6\xF5"
TARGET = {0: "none", 1: "moving", 2: "static", 3: "both"}


# ── STAGE 1 : is anything arriving at all? ───────────────
def stage1(ser, seconds=5):
    print(f"\n[1] raw bytes for {seconds}s @ {BAUD} baud")
    print("    F4 F3 F2 F1 = frame header\n")
    t0, total = time.time(), 0
    while time.time() - t0 < seconds:
        d = ser.read(ser.in_waiting or 1)
        if d:
            total += len(d)
            print(" ".join(f"{b:02X}" for b in d))
    print(f"\n    {total} bytes in {seconds}s")
    if total == 0:
        print("    >>> NOTHING. wiring / power / wrong port.")
    elif HDR not in b"":
        print("    >>> bytes arriving. look for F4 F3 F2 F1 above.")
        print("    >>> no header = alive but WRONG BAUD.")
    return total > 0


# ── STAGE 2 : find and validate frames ───────────────────
def read_frame(ser, buf):
    """returns (data, buf) or (None, buf)"""
    while True:
        i = buf.find(HDR)
        if i < 0:
            buf = buf[-4:] + ser.read(ser.in_waiting or 1)
            continue
        if len(buf) < i + 6:
            buf += ser.read(ser.in_waiting or 1)
            continue
        n   = int.from_bytes(buf[i+4:i+6], "little")
        end = i + 6 + n + 4
        if len(buf) < end:
            buf += ser.read(ser.in_waiting or 1)
            continue
        frame, buf = buf[i:end], buf[end:]
        if frame[-4:] != TAIL:
            continue                      # bad tail, resync
        return frame[6:6+n], buf


def stage2(ser, n=5):
    print(f"\n[2] validating {n} frames")
    buf = b""
    for _ in range(n):
        data, buf = read_frame(ser, buf)
        print(f"    len={len(data):2d}  type=0x{data[0]:02X}  "
              f"{' '.join(f'{b:02X}' for b in data)}")


# ── STAGE 3 : parse target data ──────────────────────────
def stage3(ser):
    print("\n[3] parsed output — Ctrl+C to stop\n")
    buf = b""
    while True:
        data, buf = read_frame(ser, buf)
        if len(data) < 9 or data[0] != 0x02:
            continue                      # 0x02 = basic target frame
        state   = data[2]
        move_cm = int.from_bytes(data[3:5], "little")
        move_e  = data[5]
        stat_cm = int.from_bytes(data[6:8], "little")
        stat_e  = data[8]
        print(f"{TARGET.get(state,'?'):7s}  "
              f"moving {move_cm:4d} cm (e{move_e:3d})   "
              f"static {stat_cm:4d} cm (e{stat_e:3d})")


if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        sys.exit(f"cannot open {PORT}: {e}")

    stage = sys.argv[1] if len(sys.argv) > 1 else "1"
    try:
        if stage == "1": stage1(ser)
        elif stage == "2": stage2(ser)
        else: stage3(ser)
    except KeyboardInterrupt:
        print("\nstopped")