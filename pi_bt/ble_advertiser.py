#!/usr/bin/env python3

import subprocess
import threading
import queue
import time

class BLEAdvertiserThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()
        self._payload_queue = queue.Queue()
        self._current_payload = "B1:0,9.00,000,000"
        self.count = 0
        self.int_part = 9
        self.frac_part = 0
        self.pan = 0  # pan
        self.tilt = 0  # tilt

    def advertise_data(self, custom_str):
        ascii_bytes = [f"{ord(c):02X}" for c in custom_str]
        hex_str = " ".join(ascii_bytes)

        manuf_len = len(ascii_bytes) + 3
        total_len = 3 + 1 + manuf_len

        full_cmd = f"sudo hcitool -i hci0 cmd 0x08 0x0008 " \
                   f"{total_len:02X} 02 01 06 " \
                   f"{manuf_len:02X} FF FF FF {hex_str}"

        subprocess.run(full_cmd.split(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def enable_advertising(self):
        subprocess.run(["sudo", "hciconfig", "hci0", "up"])
        subprocess.run([
            "sudo", "hcitool", "-i", "hci0", "cmd", "0x08", "0x0006",
            "0A", "00", "0A", "00", "03", "00", "00",
            "00", "00", "00", "00", "00", "00",
            "07", "00"
        ])
        subprocess.run(["sudo", "hciconfig", "hci0", "leadv", "0"])

    def disable_advertising(self):
        subprocess.run(["sudo", "hciconfig", "hci0", "noleadv"])

    def _build_payload(self):
        return f"B1:{self.count},{self.int_part}.{self.frac_part:02d},{self.pan:04d},{self.tilt:04d}"

    def update_payload(self, new_payload):
        self._payload_queue.put(new_payload)
    
    def update_count(self, count):
        self.count = count
        self.update_payload(self._build_payload())

    def update_integer(self, integer):
        self.int_part = integer
        self.update_payload(self._build_payload())

    def update_frac(self, frac):
        self.frac_part = frac
        self.update_payload(self._build_payload())
    
    def update_pan(self, pan):
        self.pan = pan
        self.update_payload(self._build_payload())

    def update_tilt(self, tilt):
        self.tilt = tilt
        self.update_payload(self._build_payload())

    def run(self):
        self.enable_advertising()
        print("[BLE] Advertising started.")

        while not self._stop_event.is_set():
            try:
                new_payload = self._payload_queue.get(timeout=1)
                self._current_payload = new_payload
            except queue.Empty:
                pass  # reuse last payload

            if len(self._current_payload) > 26:
                print(f"[BLE] WARN: Payload too long ({len(self._current_payload)}): {self._current_payload}")
            else:
                print(f"[BLE] Advertising: {self._current_payload}")
                self.advertise_data(self._current_payload)

        self.disable_advertising()
        print("[BLE] Advertising stopped.")

    def stop(self):
        self._stop_event.set()

# Example usage
if __name__ == "__main__":
    advertiser = BLEAdvertiserThread()
    advertiser.start()

    try:
        count = 0
        while True:
            int_part = count % 10
            frac_part = (count * 7) % 100
            val1 = (count * 3) % 1000
            val2 = (count * 5) % 1000

            payload = f"B1:{count},{int_part}.{frac_part:02d},{val1:04d},{val2:04d}"
            advertiser.update_payload(payload)
            time.sleep(1)
            count += 1
    except KeyboardInterrupt:
        print("\n[MAIN] Stopping advertiser...")
        advertiser.stop()
        advertiser.join()
