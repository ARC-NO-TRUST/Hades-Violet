#!/usr/bin/env python3

import subprocess
import time

def advertise_data(custom_str):
    ascii_bytes = [f"{ord(c):02X}" for c in custom_str]
    hex_str = " ".join(ascii_bytes)

    # Manufacturer-specific field length = 3 bytes (FF FF FF) + data
    manuf_len = len(ascii_bytes) + 3

    # Total payload = flags (3 bytes) + manufacturer field
    total_len = 3 + 1 + manuf_len  # 02 01 06 + 1-byte length prefix + manuf data

    full_cmd = f"sudo hcitool -i hci0 cmd 0x08 0x0008 " \
               f"{total_len:02X} 02 01 06 " \
               f"{manuf_len:02X} FF FF FF {hex_str}"

    subprocess.run(full_cmd.split())


def enable_advertising():
    subprocess.run(["sudo", "hciconfig", "hci0", "up"])

    # Set fast advertising interval: 0x0014 = 20ms
    # cmd 0x08 0x0006: Set Advertising Parameters
    subprocess.run([
        "sudo", "hcitool", "-i", "hci0", "cmd", "0x08", "0x0006",
        "14", "00", "14", "00",  # min & max interval = 20ms
        "00",                    # adv type: ADV_IND
        "00",                    # own address type: public
        "00",                    # direct addr type
        "00", "00", "00", "00", "00", "00",  # direct addr (not used)
        "07",                    # adv channel map (all channels)
        "00"                     # filter policy: allow all
    ])

    # Enable advertising
    subprocess.run(["sudo", "hciconfig", "hci0", "leadv", "0"])


def disable_advertising():
    subprocess.run(["sudo", "hciconfig", "hci0", "noleadv"])

def main():
    enable_advertising()
    print("Advertisement started. Updating payload every 10 seconds. Ctrl+C to stop.")

    try:
        count = 0
        while True:
            # Simulate dynamic data (you can replace this with sensor data or input)
            distance = round(1.23 + count * 0.5, 2)
            payload = f"B1: {count},{distance:.2f}"
            print(f"Advertising payload: {payload}")
            advertise_data(payload)
            count += 1
            time.sleep(1)
    except KeyboardInterrupt:
        disable_advertising()
        print("\nAdvertisement stopped.")

if __name__ == "__main__":
    main()