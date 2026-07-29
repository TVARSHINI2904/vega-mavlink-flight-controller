#!/usr/bin/env python3
# Usage: python3 set_msig_and_auth.py --connect <CONN_STR> --mission-id <ID> --mission-ver <VER> mission.csv
# Example CONN_STR: 'tcp:127.0.0.1:5760' or 'serial:/dev/ttyUSB0:57600'
#
# This script:
# 1. Reads waypoints from CSV (optional validation only)
# 2. Sets CMD_AUTH for secure command auth
# 3. Sets MISSION_ID and MISSION_VER parameters for mission upload protection
# 4. After this, upload the mission from your GCS and answer the challenge

import sys, csv, argparse, struct
from pymavlink import mavutil


def int_to_float_bits(value):
    return struct.unpack('<f', struct.pack('<I', value))[0]


def read_csv(path):
    wps = []
    with open(path, newline='') as f:
        r = csv.reader(f)
        for row in r:
            lat = float(row[0])
            lon = float(row[1])
            alt_m = float(row[2])
            cmd = int(row[3])
            # convert to vehicle internal formats: deg -> degE7, alt m -> float bits (IEEE754)
            # Use float32 rounding to match firmware/Mission Planner behavior exactly.
            lat_f = struct.unpack('<f', struct.pack('<f', float(lat)))[0]
            lon_f = struct.unpack('<f', struct.pack('<f', float(lon)))[0]
            lat_i = int(lat_f * 10000000.0)
            lon_i = int(lon_f * 10000000.0)
            alt_bits = struct.unpack('<I', struct.pack('<f', float(alt_m)))[0]
            wps.append((lat_i, lon_i, alt_bits, cmd))
    return wps

def build_bytes(wps):
    b = bytearray()
    for (lat_i, lon_i, alt_bits, cmd) in wps:
            # pack little-endian: int32, int32, uint32, uint16, 2 bytes padding -> total 16 bytes
        b += struct.pack('<iiI', lat_i, lon_i, alt_bits)
        b += struct.pack('<H', cmd)
        b += b'\x00\x00'  # padding to match sizeof(waypoint_t)==16
    return bytes(b)

def send_param_and_wait(conn, name, value, param_type, timeout=5):
    """Send a PARAM_SET and wait for PARAM_VALUE acknowledgment."""
    print(f'  Setting {name} = {value} ...', end=' ', flush=True)
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        name.encode('ascii'), float(value), param_type
    )
    # Wait for PARAM_VALUE ACK
    ack = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
    if ack is None:
        print('TIMEOUT - no ACK received!')
        return False
    # Verify the param_id matches
    ack_name = ack.param_id.rstrip('\x00')
    if ack_name != name:
        print(f'WRONG ACK - got {ack_name}')
        return False
    print(f'OK (value={ack.param_value})')
    return True

def send_params(conn_str, mission_id, mission_ver, auth_token=1234.0):
    conn = mavutil.mavlink_connection(conn_str)
    conn.wait_heartbeat(timeout=10)
    print(f'Connected: sysid={conn.target_system} compid={conn.target_component}')
    
    if not send_param_and_wait(conn, 'CMD_AUTH', auth_token, mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        print('WARNING: CMD_AUTH may not be set!')

    id_val = int_to_float_bits(mission_id)
    ver_val = int_to_float_bits(mission_ver)
    all_ok = True
    if not send_param_and_wait(conn, 'MISSION_ID', id_val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        all_ok = False
    if not send_param_and_wait(conn, 'MISSION_VER', ver_val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
        all_ok = False

    if all_ok:
        print('\n✅ CMD_AUTH, MISSION_ID, and MISSION_VER set successfully.')
        print('Now initiate mission upload from your GCS. Once MISSION_CHALLENGE appears, set MISSION_CHAL_RESP to match it.')
    else:
        print('\n❌ Some params failed. Check connection and try again.')

    conn.close()

def main():
    p = argparse.ArgumentParser(description='Set mission upload auth params (MISSION_ID/MISSION_VER and CMD_AUTH)')
    p.add_argument('--connect', required=True, help='Connection string (e.g. tcp:127.0.0.1:5760)')
    p.add_argument('--mission-id', type=int, default=1, help='Mission ID value encoded as float bits')
    p.add_argument('--mission-ver', type=int, default=1, help='Mission version value encoded as float bits')
    p.add_argument('--auth', type=float, default=1234.0, help='CMD_AUTH token value (default: 1234.0)')
    p.add_argument('mission_csv', help='Path to CSV file with waypoints')
    args = p.parse_args()

    wps = read_csv(args.mission_csv)
    print(f'Read {len(wps)} waypoints from {args.mission_csv}')
    print(f'Setting mission id={args.mission_id}, version={args.mission_ver}')
    print()
    
    send_params(args.connect, args.mission_id, args.mission_ver, auth_token=args.auth)

if __name__ == '__main__':
    main()