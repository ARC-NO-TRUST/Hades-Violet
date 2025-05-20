import serial
import time
import struct

# Config command UART
CLI_PORT = '/dev/ttyUSB0'
# Data stream UART
DATA_PORT = '/dev/ttyUSB1'
# Path to .cfg file
CFG_FILE = 'config.cfg'

def send_config(cli_serial, cfg_file_path):
    print("Sending config to radar...")
    with open(cfg_file_path, 'r') as cfg_file:
        for line in cfg_file:
            line = line.strip()
            if not line or line.startswith('%'):
                continue
            cli_serial.write((line + '\n').encode())
            print("→", line)
            time.sleep(0.05)
            while cli_serial.in_waiting:
                resp = cli_serial.readline().decode(errors='ignore').strip()
                print("  ←", resp)
                if "sensorStart" in line:
                    print("sensorStart sent")
                    return

def read_data(data_serial):
    print("\nReading data from radar...\n")
    MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

    while True:
        sync = data_serial.read(8)
        if sync == MAGIC_WORD:
            header = sync + data_serial.read(32)
            total_packet_len = struct.unpack('<I', header[12:16])[0]
            payload_len = total_packet_len - len(header)
            payload = data_serial.read(payload_len)
            print(f"Frame received: {total_packet_len} bytes")
            # TODO: Parse here if needed
        else:
            # Lost sync, keep searching
            continue

if __name__ == '__main__':
    # Open serial ports
    cli = serial.Serial(CLI_PORT, 115200, timeout=1)
    data = serial.Serial(DATA_PORT, 921600, timeout=0.5)

    send_config(cli, CFG_FILE)
    read_data(data)
