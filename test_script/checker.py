#!/usr/bin/env python3
import os
import fnmatch
import time
from pymavlink import mavutil

TELEM2_BAUD = 57600
HEARTBEAT_TIMEOUT = 5  # seconds

def find_serial_ports():
    """Scan /dev for potential Pixhawk ports"""
    ports = []
    for entry in os.listdir("/dev"):
        if fnmatch.fnmatch(entry, "ttyUSB*") or fnmatch.fnmatch(entry, "ttyACM*"):
            ports.append(f"/dev/{entry}")
    return ports

def check_pixhawk_telem2(port, baud=TELEM2_BAUD, timeout=HEARTBEAT_TIMEOUT):
    try:
        print(f"[Pixhawk] Trying {port} at {baud} baud...")
        master = mavutil.mavlink_connection(port, baud=baud)
        print(f"[Pixhawk] Waiting for heartbeat on {port}...")
        hb = master.wait_heartbeat(timeout=timeout)
        if hb is None:
            print(f"[Pixhawk][TIMEOUT] No heartbeat received on {port} within {timeout}s")
            return None

        print(f"[Pixhawk] Connected! System ID: {master.target_system}, Component ID: {master.target_component}")

        # Request some info for debugging
        master.mav.param_request_list_send(master.target_system, master.target_component)
        time.sleep(1)

        # Try reading a few messages
        for _ in range(5):
            msg = master.recv_match(blocking=True, timeout=2)
            if msg:
                print(f"[Pixhawk][MSG] {msg.get_type()} -> {msg.to_dict()}")
            else:
                print("[Pixhawk] No message received")
        return master

    except Exception as e:
        print(f"[Pixhawk][ERROR] Could not connect on {port}: {e}")
        return None

def main():
    ports = find_serial_ports()
    if not ports:
        print("[ERROR] No serial devices found under /dev/")
        return

    print(f"[INFO] Found serial ports: {ports}")

    for port in ports:
        pixhawk = check_pixhawk_telem2(port)
        if pixhawk:
            print(f"[SUCCESS] Pixhawk connected on {port}")
            break
    else:
        print("[FAIL] Could not connect to Pixhawk on any port")

if __name__ == "__main__":
    main()
