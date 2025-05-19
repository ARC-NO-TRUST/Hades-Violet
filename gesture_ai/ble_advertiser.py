#!/usr/bin/env python3

import subprocess
import time

def advertise_data(custom_str):
    # Convert ASCII string to hex bytes
    ascii_bytes = [f"{ord(c):02X}" for c in custom_str]
    hex_str = " ".join(ascii_bytes)

    # Build the full HCI command string
    full_cmd = f"sudo hcitool -i hci0 cmd 0x08 0x0008 " \
               f"{(len(ascii_bytes) + 5):02X} 02 01 06 " \
               f"{(len(ascii_bytes) + 1):02X} FF FF FF {hex_str}"

    # Enable Bluetooth interface and advertising
    subprocess.run(["sudo", "hciconfig", "hci0", "up"])
    subprocess.run(["sudo", "hciconfig", "hci0", "leadv", "0"])
    subprocess.run(full_cmd.split())

def main():
    # Example payload — change this dynamically if needed
    payload = "B1: 1,2.30"
    print(f"Advertising payload: {payload}")
    advertise_data(payload)
    print("Advertisement started. Ctrl+C to stop.")

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        subprocess.run(["sudo", "hciconfig", "hci0", "noleadv"])
        print("\nAdvertisement stopped.")

if __name__ == "__main__":
    main()
