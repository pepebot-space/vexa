"""
YDLIDAR X2 360° Lidar Scanner - UART Reader

Protocol: YDLIDAR X2 uses laser triangulation with the following packet format:
  Header: 0xAA 0x55
  [CT: 1byte] [LSN: 1byte] [FSA: 2bytes] [LSA: 2bytes] [CS: 2bytes] [Data: LSN*2 bytes]

  CT  - Package type. Bit0: 0 = normal point cloud, 1 = start of new scan
  LSN - Number of samples in this packet
  FSA - First Sample Angle (start angle)
  LSA - Last Sample Angle (end angle)
  CS  - Checksum (XOR of all preceding bytes as uint16)
  Data - Distance samples, 2 bytes each (little-endian, in mm)

Serial: 115200 baud, 8N1
"""

import struct
import sys
import time
import math
import serial

SERIAL_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 115200
HEADER = bytes([0xAA, 0x55])


def calc_angle(raw_angle):
    """Convert raw FSA/LSA value to degrees. Angle = (raw >> 1) / 64.0"""
    return (raw_angle >> 1) / 64.0


def parse_packet(data):
    """
    Parse a single lidar data packet.
    Returns list of (angle_deg, distance_mm) tuples.
    """
    if len(data) < 10:
        return []

    ct = data[0]
    lsn = data[1]
    fsa = struct.unpack_from("<H", data, 2)[0]
    lsa = struct.unpack_from("<H", data, 4)[0]
    # cs = struct.unpack_from("<H", data, 6)[0]  # checksum

    start_angle = calc_angle(fsa)
    end_angle = calc_angle(lsa)

    # Calculate angle difference (handle wrap-around)
    if end_angle < start_angle:
        angle_diff = (360.0 - start_angle) + end_angle
    else:
        angle_diff = end_angle - start_angle

    expected_data_len = lsn * 2
    sample_data = data[8:]

    if len(sample_data) < expected_data_len:
        return []

    points = []
    for i in range(lsn):
        distance = struct.unpack_from("<H", sample_data, i * 2)[0]

        # Calculate interpolated angle for this sample
        if lsn > 1:
            angle = start_angle + (angle_diff / (lsn - 1)) * i
        else:
            angle = start_angle

        angle = angle % 360.0
        points.append((angle, distance))

    is_start = (ct & 0x01) == 1
    return points, is_start


def read_packet(ser):
    """
    Read one complete packet from serial.
    Syncs on 0xAA 0x55 header, then reads the rest.
    """
    # Sync to header
    while True:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == 0xAA:
            b2 = ser.read(1)
            if b2 and b2[0] == 0x55:
                break

    # Read CT (1) + LSN (1) + FSA (2) + LSA (2) + CS (2) = 8 bytes
    header_data = ser.read(8)
    if len(header_data) < 8:
        return None

    lsn = header_data[1]  # number of samples
    data_len = lsn * 2

    # Read sample data
    sample_data = ser.read(data_len)
    if len(sample_data) < data_len:
        return None

    return header_data + sample_data


def print_scan(scan_points):
    """Print a completed 360° scan summary."""
    if not scan_points:
        return

    distances = [d for _, d in scan_points if d > 0]
    if not distances:
        print("[!] Scan completed but no valid distances")
        return

    min_d = min(distances)
    max_d = max(distances)
    avg_d = sum(distances) / len(distances)

    print(f"\n{'='*60}")
    print(f"  SCAN: {len(scan_points)} points | "
          f"Valid: {len(distances)} | "
          f"Min: {min_d}mm | Max: {max_d}mm | Avg: {avg_d:.0f}mm")
    print(f"{'='*60}")

    # Simple ASCII top-down view (12 sectors of 30°)
    sectors = {}
    for angle, dist in scan_points:
        if dist > 0:
            sector = int(angle // 30)
            if sector not in sectors:
                sectors[sector] = []
            sectors[sector].append(dist)

    labels = ["  0°(F)", " 30°", " 60°", " 90°(R)", "120°", "150°",
              "180°(B)", "210°", "240°", "270°(L)", "300°", "330°"]

    for i in range(12):
        if i in sectors:
            avg = sum(sectors[i]) / len(sectors[i])
            bar_len = min(int(avg / 100), 40)  # 100mm per char, max 40
            bar = "█" * bar_len
            print(f"  {labels[i]:>8s} | {bar} {avg:.0f}mm ({len(sectors[i])}pts)")
        else:
            print(f"  {labels[i]:>8s} | -- no data --")

    print()


def main():
    print(f"[*] YDLIDAR X2 360° Reader")
    print(f"[*] Port: {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"[*] Press Ctrl+C to stop\n")

    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )
    except serial.SerialException as e:
        print(f"[!] Cannot open serial port: {e}")
        sys.exit(1)

    print(f"[+] Connected! Waiting for data...\n")

    scan_points = []
    scan_count = 0

    try:
        while True:
            raw = read_packet(ser)
            if raw is None:
                continue

            result = parse_packet(raw)
            if result is None:
                continue

            points, is_start = result

            if is_start and scan_points:
                # New scan started - print previous scan
                scan_count += 1
                print_scan(scan_points)
                scan_points = []

            scan_points.extend(points)

    except KeyboardInterrupt:
        print(f"\n[*] Stopped. Total scans: {scan_count}")
    finally:
        ser.close()
        print("[*] Serial port closed.")


if __name__ == "__main__":
    main()
