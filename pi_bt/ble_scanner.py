from bluepy.btle import Scanner, DefaultDelegate
import threading
import time
import binascii

TARGET_MAC = "c8:ae:54:01:ac:a9"

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        super().__init__()

class BLEScannerThread(threading.Thread):
    def __init__(self, scan_interval=5.0, pause_interval=0.5):
        super().__init__()
        self.scanner = Scanner().withDelegate(ScanDelegate())
        self.scan_interval = scan_interval
        self.pause_interval = pause_interval
        self._stop_event = threading.Event()

    def decode_data(self, adtype, desc, value):
        if desc == "Manufacturer":
            try:
                raw_bytes = bytes.fromhex(value)
                payload = raw_bytes[:11]

                try:
                    decoded = payload.decode("utf-8").strip()
                    print(f"    ▸ Payload (UTF-8): {decoded}")
                except UnicodeDecodeError:
                    print(f"    ▸ Payload (hex)  : {payload.hex()}")

            except Exception as e:
                print(f"    ▸ Decode error: {e}")
        elif desc == "Complete Local Name":
            print(f"    ▸ Device Name: {value}")
        else:
            print(f"    ▸ {desc}: {value}")


    def run(self):
        print("[SCANNER] Thread started – press Ctrl-C to stop")
        while not self._stop_event.is_set():
            try:
                print("[SCANNER] Scanning for BLE devices...")
                devices = self.scanner.scan(self.scan_interval)

                for dev in devices:
                    if dev.addr.lower() == TARGET_MAC:
                        print(f"\n[FOUND] Device {dev.addr} (RSSI={dev.rssi} dB)")
                        for (adtype, desc, value) in dev.getScanData():
                            self.decode_data(adtype, desc, value)

                print(f"[SCANNER] Sleeping {self.pause_interval} seconds...\n")
                time.sleep(self.pause_interval)
            except Exception as e:
                print(f"[SCANNER] Error: {e}")

        print("[SCANNER] Thread stopped.")

    def stop(self):
        self._stop_event.set()

# Example usage
if __name__ == "__main__":
    scanner_thread = BLEScannerThread(scan_interval=5.0, pause_interval=1.0)
    scanner_thread.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[MAIN] Stopping scanner thread...")
        scanner_thread.stop()
        scanner_thread.join()
