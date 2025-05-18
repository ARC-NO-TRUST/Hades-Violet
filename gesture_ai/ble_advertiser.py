#!/usr/bin/env python3
import socket
import struct
import time
import sys  # ← ✅ this is what you're missing
import fcntl
import os

HCI_DEV = 0  # hci0
OGF_LE_CTL = 0x08
OCF_LE_SET_ADV_PARAM = 0x0006
OCF_LE_SET_ADV_DATA = 0x0008
OCF_LE_SET_ADV_ENABLE = 0x000A


def hci_send_cmd(sock, ogf, ocf, data):
    opcode = (ocf & 0x03FF) | (ogf << 10)
    cmd = struct.pack("<BHB", 0x01, opcode, len(data)) + data
    sock.send(cmd)

def str_to_adv_data(s: str) -> bytes:
    b = s.encode("ascii")
    company_id = b'\xFF\xFF'
    full_payload = company_id + b
    if len(full_payload) + 1 > 28:  # 31 - 3 (Flags AD)
        raise ValueError("Payload too long for BLE advertising")

    length = len(full_payload) + 1  # +1 for AD type
    return struct.pack("B", length) + b'\xFF' + full_payload

def main(payload: str, interval_ms: int = 100):
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI)
    sock.bind((HCI_DEV,))

    flags = b'\x02\x01\x06'
    mfg_data = str_to_adv_data(payload)
    adv_data = flags + mfg_data

    if len(adv_data) > 31:
        raise ValueError("adv_data too long")

    print("adv_data:", adv_data.hex())

    # 1. Disable advertising first
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADV_ENABLE, b'\x00')

    # 2. Set advertising parameters
    iv = int(interval_ms / 0.625)
    params = struct.pack("<HHBBB6sBB",
        iv, iv,
        0x00,         # ADV_IND
        0x00,         # public addr
        0x00,         # direct addr type
        b'\x00' * 6,
        0x07,         # all channels
        0x00          # allow all
    )
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADV_PARAM, params)

    # 3. Set actual advertising data (short form)
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADV_DATA, adv_data)

    # 4. Re-enable advertising
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

