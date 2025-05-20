import serial
import serial.tools.list_ports
import time
import struct
import sys

CFG_FILE = 'config.cfg'

def detect_ports_linux():
    ports = [p.device for p in serial.tools.list_ports.comports() if 'ttyACM' in p.device]
    if len(ports) < 2:
        print("Could not detect both CLI and DATA ports (ACM). Found:", ports)
        sys.exit(1)

    ports.sort()  # Ensure ttyACM0 is CLI and ttyACM1 is DATA
    cli_port = ports[0]
    data_port = ports[1]

    print(f"CLI Port:  {cli_port}")
    print(f"Data Port: {data_port}")
    return cli_port, data_port

def send_config(cli_serial, cfg_file_path):
    print("Sending config to radar...")
    with open(cfg_file_path, 'r') as cfg_file:
        for line in cfg_file:
            line = line.strip()
            if not line or line.startswith('%'):
                continue
            cli_serial.write((line + '\n').encode())
            print("->", line)
            time.sleep(0.05)
            while cli_serial.in_waiting:
                resp = cli_serial.readline().decode(errors='ignore').strip()
                print("  <-", resp)
                if "sensorStart" in line:
                    print("sensorStart sent")
                    return

def parse_tlv_payload(payload):
    idx = 0
    while idx + 8 <= len(payload):
        try:
            tlv_type, tlv_length = struct.unpack_from('<II', payload, idx)
        except Exception as e:
            print(f"[TLV PARSE ERROR] Failed to unpack: {e}")
            break

        idx += 8
        print(f"\nTLV type: {tlv_type}, length: {tlv_length}")

        if tlv_length < 8 or tlv_length > len(payload) - (idx - 8):
            print("Invalid TLV length. Skipping.")
            continue

        tlv_payload = payload[idx:idx + tlv_length - 8]

        if tlv_type == 1:
            print("Detected object TLV (type 1) — implement decoder here.")
            # placeholder: implement object decoding logic
        elif tlv_type == 7:
            print("Stats or noise floor TLV — skipping or log info.")
        else:
            print("Unknown or unsupported TLV type. Skipping.")

        idx += tlv_length - 8

def read_data(data_serial):
    print("\nReading data from radar...\n")
    MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

    while True:
        sync = data_serial.read(8)
        if sync != MAGIC_WORD:
            continue

        header = sync + data_serial.read(32)
        total_packet_len = struct.unpack('<I', header[12:16])[0]

        if total_packet_len < 40 or total_packet_len > 8192:
            print(f"Invalid total_packet_len: {total_packet_len}. Skipping.")
            continue

        payload_len = total_packet_len - len(header)
        payload = data_serial.read(payload_len)

        if len(payload) != payload_len:
            print(f"Incomplete payload: expected {payload_len}, got {len(payload)}. Skipping frame.")
            continue

        print(f"\nFrame received: {total_packet_len} bytes")
        parse_tlv_payload(payload)

if __name__ == '__main__':
    cli_port, data_port = detect_ports_linux()
    cli = serial.Serial(cli_port, 115200, timeout=1)
    data = serial.Serial(data_port, 921600, timeout=0.5)

    send_config(cli, CFG_FILE)
    read_data(data)