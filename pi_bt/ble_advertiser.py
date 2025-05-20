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

    # Always use ADV_NONCONN_IND (0x03) with 10 ms interval (0x000A)
    subprocess.run([
        "sudo", "hcitool", "-i", "hci0", "cmd", "0x08", "0x0006",
        "0A", "00",  # Min interval = 10 ms
        "0A", "00",  # Max interval = 10 ms
        "03",        # Advertising type = ADV_NONCONN_IND (non-connectable)
        "00",        # Own address type = public
        "00",        # Direct address type (not used)
        "00", "00", "00", "00", "00", "00",  # Direct address
        "07",        # Advertising channels (all)
        "00"         # Filter policy = allow all
    ])

    # Enable advertising
    subprocess.run(["sudo", "hciconfig", "hci0", "leadv", "0"])


def disable_advertising():
    subprocess.run(["sudo", "hciconfig", "hci0", "noleadv"])

def main():
    enable_advertising()
    print("Advertisement started. Updating payload every 1 second. Ctrl+C to stop.")

    try:
        count = 0
        while True:
            # Simulated data values
            int_part = count % 10            # e.g., 1
            frac_part = (count * 7) % 100     # e.g., 23
            val1 = (count * 3) % 1000         # e.g., 000–999
            val2 = (count * 5) % 1000         # e.g., 000–999

            payload = f"B1:{count},{int_part}.{frac_part:02d},{val1:03d},{val2:03d}"

            # Check length safety
            if len(payload) > 24:
                print(f"[WARN] Payload too long ({len(payload)} bytes): {payload}")
            else:
                print(f"Advertising payload: {payload}")
                advertise_data(payload)

            count += 1
            time.sleep(1)
    except KeyboardInterrupt:
        disable_advertising()
        print("\nAdvertisement stopped.")


if __name__ == "__main__":
    main()