#!/usr/bin/env python3
import socket
import struct
import time
import sys
import struct
import fcntl
import os

# from bluetooth._bluetooth import *
HCI_DEV = 0  # hci0
OGF_LE_CTL       = 0x08
OCF_LE_SET_ADV_DATA      = 0x0008
OCF_LE_SET_ADV_PARAM     = 0x0006
OCF_LE_SET_ADV_ENABLE    = 0x000A

def hci_send_cmd(sock, ogf, ocf, data):
    """Pack and send an HCI command over the control socket."""
    # Build the 2-byte opcode: lower 10 bits = OCF, upper 6 bits = OGF
    opcode = (ocf & 0x03FF) | (ogf << 10)
    # 0x01 = HCI Command Packet indicator
    # Packet layout: [ 0x01 | opcode (LE) | data_len | data... ]
    cmd_pkt = struct.pack("<BHB", 0x01, opcode, len(data)) + data
    sock.send(cmd_pkt)

def str_to_adv_data(s: str) -> bytes:
    b = s.encode("ascii")
    company_id = b'\xFF\xFF'
    payload = company_id + b
    length = len(payload) + 1
    return struct.pack("B", length) + b'\xFF' + payload

def main(payload: str, interval_ms: int = 100):
    # 1) open a raw HCI socket
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI)
    sock.bind((HCI_DEV,))

    # 2) Build full 31-byte ADV_DATA buffer:
    #    - flags AD (0x01): 0x06 = GENERAL_DISCOVERABLE + NO_BREDR
    flags = b'\x02\x01\x06'
    mfg_data = str_to_adv_data(payload)
    adv_data = (flags + mfg_data).ljust(31, b'\x00')

    # 3) LE Set Advertising Data
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADV_DATA, adv_data)

    # 4) LE Set Advertising Parameters
    #    interval in units of 0.625ms
    iv = int(interval_ms / 0.625)
    iv_le = struct.pack("<HH", iv, iv)
    params = (
        iv_le +                # min/max interval
        b'\x00' +              # adv type = ADV_IND
        b'\x00' +              # own addr type = PUBLIC
        b'\x00' +              # direct addr type
        b'\x00'*6 +            # direct addr
        b'\x07' +              # adv channel map = all
        b'\x00'                # filter policy = no whitelist
    )
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADV_PARAM, params)

    # 5) Enable advertising
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADV_ENABLE, b'\x01')

    print(f"Advertising payload: {payload!r} (interval {interval_ms}ms)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nDisabling…")
        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADV_ENABLE, b'\x00')
        sock.close()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <your-payload-string>")
        sys.exit(1)
    main(sys.argv[1])
