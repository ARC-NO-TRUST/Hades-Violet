#!/usr/bin/env python3
import socket
import struct
import time

def hci_send_cmd(sock, ogf, ocf, data):
    opcode = (ocf & 0x03FF) | (ogf << 10)
    cmd = struct.pack("<BHB", 0x01, opcode, len(data)) + data
    sock.send(cmd)

def main():
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI)
    sock.bind((0,))  # hci0

    # Build advertising payload
    flags = b'\x02\x01\x06'                         # Flags AD
    mfg_data = b'\x06\xff\xff\xffHi'                # len=6, type=0xFF, ID=0xFFFF, payload="Hi"
    adv_data = flags + mfg_data
    print("adv_data:", adv_data.hex())              # Confirm what's being sent

    hci_send_cmd(sock, 0x08, 0x0008, adv_data)       # Set advertising data

    # Set parameters and enable
    interval = int(100 / 0.625)
    iv_bytes = struct.pack("<HH", interval, interval)
    params = iv_bytes + b'\x00' + b'\x00' + b'\x00'*6 + b'\x07' + b'\x00'
    hci_send_cmd(sock, 0x08, 0x0006, params)         # Set advertising parameters
    hci_send_cmd(sock, 0x08, 0x000A, b'\x01')        # Enable advertising

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        hci_send_cmd(sock, 0x08, 0x000A, b'\x00')    # Disable advertising
        sock.close()

if __name__ == "__main__":
    main()
