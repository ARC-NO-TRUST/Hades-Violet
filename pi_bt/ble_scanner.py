from bluepy.btle import Scanner, DefaultDelegate
import threading
import time
import binascii
import queue

MOBILE_MAC = "c8:ae:54:01:ac:a9"
ULTRASONIC_MAC = "fd:e2:2a:ba:f9:f1"
ACTUATOR_MAC = "f1:63:1e:d2:07:0f"

MOBILE_NAME = "MOBILE"
ULTRASONIC_NAME = "ULTRASONIC"
ACTUATOR_NAME = "ACTUATOR"

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        super().__init__()

class BLEScannerThread(threading.Thread):
    def __init__(self, scan_interval=5.0, pause_interval=0.5):
        super().__init__()
        self._data_queue = queue.Queue()  # shared queue
        self.scanner = Scanner().withDelegate(ScanDelegate())
        self.scan_interval = scan_interval
        self.pause_interval = pause_interval
        self._stop_event = threading.Event()
    
    def get_queue(self):
        return self._data_queue

    def add_to_queue(self, name, addr, data):
        self._data_queue.put({"name": name, "addr":addr, "value":data})
    
    def extract_payload(self, value):
        try:
            raw_bytes = bytes.fromhex(value)
            payload = raw_bytes  
            try:
                return {"value": payload.decode("utf-8").strip(), "encoding": "utf-8"}
            except UnicodeDecodeError:
                return {"value": payload.hex(), "encoding": "hex"}
        except Exception as e:
            print(f"    ▸ Decode error: {e}")
            return None


    def handle_advertisement_data(self, adtype, desc, value, dev_name, dev_addr):
        if desc == "Manufacturer":
            result = self.extract_payload(value)
            if result:
                if result["encoding"] == "utf-8":
                    print(f"    ▸ Payload (UTF-8): {result['value']}")
                else:
                    print(f"    ▸ Payload (hex)  : {result['value']}")
                self.add_to_queue(dev_name, dev_addr, result["value"])
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
                    dev_addr = dev.addr
                    if dev_addr.lower() == MOBILE_MAC:
                        print(f"\n[FOUND] THINGY52: Device {dev_addr} (RSSI={dev.rssi} dB)")
                        for (adtype, desc, value) in dev.getScanData():
                            self.handle_advertisement_data(adtype, desc, value, "MOBILE", dev_addr)
                    elif dev_addr.lower() == ULTRASONIC_MAC:
                        print(f"\n[FOUND] NRF_ULTRASONIC: Device {dev_addr} (RSSI={dev.rssi} dB)")
                        for (adtype, desc, value) in dev.getScanData():
                            self.handle_advertisement_data(adtype, desc, value, "ULTRASONIC", dev_addr)
                    elif dev_addr.lower() == ACTUATOR_MAC:
                        print(f"\n[FOUND] NRF_ACTUATOR: Device {dev_addr} (RSSI={dev.rssi} dB)")
                        for (adtype, desc, value) in dev.getScanData():
                            self.handle_advertisement_data(adtype, desc, value, "ACTUATOR", dev_addr)
                    else:
                        print(f"\n[FOUND] UNKNOWN DEVICE: {dev_addr} (RSSI={dev.rssi} dB)")
                        for (adtype, desc, value) in dev.getScanData():
                            print(f"    ▸ {desc}: {value}")

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

    print("[MAIN] Scanner thread started. Waiting for BLE advertisements...\n")

    try:
        while True:
            try:
                data = scanner_thread.get_queue().get(timeout=1.0)
                name = data["name"]
                addr = data["addr"]
                value = data["value"]
                print(f"[MAIN] Received from {name} ({addr}): {value}\n")
            except queue.Empty:
                print("[MAIN] No BLE data received in this interval.")
    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt received. Stopping scanner thread...")
        scanner_thread.stop()
        scanner_thread.join()
        print("[MAIN] Scanner thread stopped. Exiting.")

