#!/usr/bin/env python3
"""Read orientation (roll/pitch/yaw) from an Xsens MTi-630 over USB.

Implements the official Xsens "MT Low-Level Communication Protocol" (XBus /
MTData2). Reference: MT_Low-Level_Documentation.pdf
  - Message frame:  Preamble(0xFA) BID(0xFF) MID LEN DATA CHECKSUM
                    checksum valid when (sum(BID..CHECKSUM) & 0xFF) == 0
  - MTData2 MID  =  0x36
  - Data element =  DataID(2B) size(1B) data
  - Data IDs (high 12 bits fixed, low nibble = format):
        Euler       0x203y  (deg)   order: Roll, Pitch, Yaw
        Quaternion  0x201y          order: q0,q1,q2,q3 (w,x,y,z)
        Accel       0x402y  (m/s^2)
        RateOfTurn  0x802y  (rad/s)
        MagField    0xC02y  (a.u.)
        Packet      0x1020  (U2)
        SampleTime  0x1060  (U4)
        StatusWord  0xE020  (U4)
  - Format nibble: precision 0=Float32 1=Fp1220 2=Fp1632 3=Float64
                   coord     0=ENU 4=NED 8=NWU   (floats are big-endian)

This is a low-level hardware reader. The clean pipeline defines session yaw at
the mechanical home pose: average the initial IMU yaw as yaw0, then use wrapped
(yaw - yaw0) for the complete session. Roll and pitch are used directly after
the rig's fixed mount/axis conversion below. Validate this convention before
recording new calibration data.

Usage:
    python3 xsens_mti630_reader.py [--port /dev/ttyUSB0] [--baud 921600]
                                   [--seconds 0] [--csv out.csv]
    --seconds 0  runs until Ctrl-C.
"""
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed. Run:  pip install --user pyserial")

PREAMBLE = 0xFA
BID = 0xFF
MID_MTDATA2 = 0x36

# high 12 bits of the data identifier -> (name, n_components)
GROUPS = {
    0x1020: ("PacketCounter", 1),
    0x1060: ("SampleTimeFine", 1),
    0x2010: ("Quaternion", 4),
    0x2030: ("EulerAngles", 3),
    0x4020: ("Acceleration", 3),
    0x4030: ("FreeAcceleration", 3),
    0x8020: ("RateOfTurn", 3),
    0xC020: ("MagneticField", 3),
    0xE020: ("StatusWord", 1),
}


def decode_floats(precision, data):
    """Decode a big-endian numeric payload per the Xsens precision nibble."""
    if precision == 0x0:                       # Float32
        n = len(data) // 4
        return list(struct.unpack(">" + "f" * n, data))
    if precision == 0x3:                       # Float64
        n = len(data) // 8
        return list(struct.unpack(">" + "d" * n, data))
    if precision == 0x1:                       # Fp12.20 (32-bit)
        out = []
        for i in range(0, len(data), 4):
            v = int.from_bytes(data[i:i + 4], "big", signed=True)
            out.append(v / (1 << 20))
        return out
    if precision == 0x2:                       # Fp16.32 (48-bit: 4B frac, 2B int)
        out = []
        for i in range(0, len(data), 6):
            frac = int.from_bytes(data[i:i + 4], "big", signed=False)
            intg = int.from_bytes(data[i + 4:i + 6], "big", signed=True)
            out.append(intg + frac / (1 << 32))
        return out
    raise ValueError(f"unknown precision {precision}")


def parse_mtdata2(payload):
    """Return {name: values} for one MTData2 payload."""
    out = {}
    i, n = 0, len(payload)
    while i + 3 <= n:
        did = (payload[i] << 8) | payload[i + 1]
        size = payload[i + 2]
        i += 3
        if i + size > n:
            break
        data = payload[i:i + size]
        i += size
        group = did & 0xFFF0
        precision = did & 0x03
        info = GROUPS.get(group)
        if info is None:
            out[f"0x{did:04X}"] = data.hex()
            continue
        name, _ = info
        if group == 0x1020:
            out[name] = struct.unpack(">H", data)[0]
        elif group in (0x1060, 0xE020):
            out[name] = struct.unpack(">I", data)[0]
        else:
            out[name] = decode_floats(precision, data)
    return out


def frames(ser):
    """Yield (mid, payload) for every checksum-valid XBus message."""
    buf = bytearray()
    while True:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
        while True:
            i = buf.find(bytes([PREAMBLE, BID]))
            if i < 0:
                if len(buf) > 4096:
                    del buf[:-2]
                break
            if len(buf) < i + 4:
                break
            mid = buf[i + 2]
            length = buf[i + 3]
            if length == 0xFF:                 # extended length not needed here
                del buf[:i + 1]
                continue
            end = i + 4 + length + 1
            if len(buf) < end:
                break
            frame = buf[i + 1:end]             # BID .. checksum
            if (sum(frame) & 0xFF) != 0:
                del buf[:i + 1]                # bad checksum, resync
                continue
            payload = bytes(buf[i + 4:i + 4 + length])
            del buf[:end]
            yield mid, payload


# The IMU is mounted flipped on the shell: 180 deg about its x (roll) axis.
# Undo it by right-multiplying every orientation with this fixed body-side
# rotation, so the printed angles are the SHELL's, reading ~0 at the home pose
# instead of ~180 roll. (Done in quaternion space -- subtracting 180 from an
# Euler angle misbehaves at the +/-180 wrap, which is exactly where we sit.)
MOUNT_QUAT = (0.0, 1.0, 0.0, 0.0)          # (w,x,y,z), 180 deg about x

# Rig sanity test (2026-07-16): after the flip correction, yaw matched the
# rig's convention (+ = anticlockwise from top) but pitch and roll came out
# sign-flipped. Conjugating by a 180 deg rotation about z negates exactly
# pitch and roll while leaving yaw untouched:  Rz(pi) R Rz(pi)^-1.
AXIS_FIX = (0.0, 0.0, 0.0, 1.0)            # (w,x,y,z), 180 deg about z


def quat_mult(a, b):
    """Hamilton product of two (w,x,y,z) quaternions."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def euler_to_quat(roll, pitch, yaw):
    """(roll,pitch,yaw) in degrees -> (w,x,y,z), zyx convention."""
    import math
    hr, hp, hy = (math.radians(a) / 2 for a in (roll, pitch, yaw))
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)
    return (cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr)


def quat_to_rpy(q):
    """q=(w,x,y,z) -> (roll, pitch, yaw) in degrees, ENU/aerospace convention."""
    import math
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="0 = run until Ctrl-C")
    ap.add_argument("--csv", default=None, help="optional CSV output path")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write("t_host,packet,roll_deg,pitch_deg,yaw_deg\n")

    print(f"# port={args.port} baud={args.baud} MID=MTData2(0x36)")
    print(f"# {'roll':>8} {'pitch':>8} {'yaw':>8}   (deg, raw session reference)")

    t0 = time.time()
    described = False
    last_print = 0.0
    try:
        for mid, payload in frames(ser):
            if mid != MID_MTDATA2:
                continue
            d = parse_mtdata2(payload)

            if not described:
                print("# data items in stream: " + ", ".join(d.keys()))
                described = True

            quat = None
            if isinstance(d.get("Quaternion"), list):
                quat = d["Quaternion"]
            elif isinstance(d.get("EulerAngles"), list):
                quat = euler_to_quat(*d["EulerAngles"])

            rpy = None
            if quat is not None:
                q = quat_mult(quat, MOUNT_QUAT)        # undo the mount flip
                q = quat_mult(AXIS_FIX, quat_mult(q, AXIS_FIX))  # flip pitch+roll signs
                rpy = quat_to_rpy(q)

            if rpy is not None:
                now = time.time()
                if csv:
                    csv.write(f"{now:.6f},{d.get('PacketCounter','')},"
                              f"{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}\n")
                if now - last_print > 0.1:
                    print(f"  {rpy[0]:8.2f} {rpy[1]:8.2f} {rpy[2]:8.2f}")
                    last_print = now

            if args.seconds and time.time() - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if csv:
            csv.close()
        ser.close()


if __name__ == "__main__":
    main()
