import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

print(f"Opening {PORT} at {BAUDRATE} baud...")
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(1)

print("Reading raw bytes from LiDAR...")
print("Looking for packet headers (0xAA 0x55)...\n")

packets_found = 0
byte_count = 0

try:
    while packets_found < 10:
        byte = ser.read(1)
        if not byte:
            continue
        
        byte_count += 1
        
        if byte[0] == 0xAA:
            next_byte = ser.read(1)
            if next_byte and next_byte[0] == 0x55:
                # Found packet header
                header = ser.read(8)
                if len(header) == 8:
                    ct = header[0]
                    lsn = header[1]
                    fsa = header[2] | (header[3] << 8)
                    lsa = header[4] | (header[5] << 8)
                    
                    start_angle = (fsa >> 1) / 64.0
                    end_angle = (lsa >> 1) / 64.0
                    
                    data_size = lsn * 3
                    data = ser.read(data_size)
                    
                    packets_found += 1
                    print(f"Packet {packets_found}:")
                    print(f"  CT=0x{ct:02X}, LSN={lsn} samples")
                    print(f"  Start angle: {start_angle:.2f}°")
                    print(f"  End angle:   {end_angle:.2f}°")
                    print(f"  Data size: {len(data)} bytes")
                    print()
        
        if byte_count > 10000:
            print("Read 10000 bytes without finding 10 packets")
            break

except KeyboardInterrupt:
    print("\nStopped")
finally:
    ser.close()
    print(f"\nTotal bytes read: {byte_count}")
    print(f"Packets found: {packets_found}")
