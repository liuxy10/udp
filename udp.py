import serial
import struct
import time
import pynput
from pynput import keyboard
from pynput.keyboard import Listener, Key
# import keyboard
# Constants
START_BYTE = 0xAA
PACKET_SIZE = 113 * 2  # Adjust if the total packet size is different
BAUD_RATE = 460800
SERIAL_PORT = '/dev/cu.usbserial-ABSCHWQ0'
log_file = 'serial_log.txt'

# Define sensor data format: (name, struct-format)
SENSOR_DATA = [
    ('TIME_SYSTEM_ON', '<I'),
    ('GAIT_PHASE', '<B'), # ('ACTIVITY', '<I'),
    ('GAIT_SUBPHASE', '<B'),  # Special handling for uint8_t
    ('ACTUATOR_POSITION', '<f'),
    ('TORQUE_ESTIMATE', '<f'),
    ('ACTUATOR_SETPOINT', '<f'),
    ('LOADCELL', '<f'),
    ('LINEAR_ACC_X_LOCAL', '<f'),
    ('LINEAR_ACC_Y_LOCAL', '<f'),
    ('LINEAR_ACC_Z_LOCAL', '<f'),
    ('GRAVITY_VECTOR_X', '<f'),
    ('GRAVITY_VECTOR_Y', '<f'),
    ('GRAVITY_VECTOR_Z', '<f'),
    ('KNEE_ANGLE', '<f'),
]

# Mapping sensor identifier (4 bytes) to sensor names
SENSOR_ID = {
    0x24115410: 'TIME_SYSTEM_ON', # int for now
    0x24111110: 'ACTIVITY', 
    0x2101C610: 'GAIT_SUBPHASE',
    0x44010770: 'ACTUATOR_POSITION',
    0x44010171: 'TORQUE_ESTIMATE',
    0x44011471: 'ACTUATOR_SETPOINT',
    0x34118830: 'LOADCELL',
    0x4401AC10: 'LINEAR_ACC_X_LOCAL',
    0x4401AD10: 'LINEAR_ACC_Y_LOCAL',
    0x4401AE10: 'LINEAR_ACC_Z_LOCAL',
    0x4401B210: 'GRAVITY_VECTOR_X',
    0x4401B310: 'GRAVITY_VECTOR_Y',
    0x4401B410: 'GRAVITY_VECTOR_Z',
    0x44118930: 'KNEE_ANGLE'
}


def parse_packet(data):
    if len(data) < PACKET_SIZE:
        return None
    
    packet = {
        'packet_type': data[1],
        'packet_id': data[2],
    }
    
    offset = 2  # sensor data section starts from this index

    for name, fmt in SENSOR_DATA:
        # Get sensor identifier (first 4 bytes for each sensor data field)
        # sensor_identifier = int.from_bytes(data[offset:offset+4], byteorder='little')
        # Optionally: you can validate sensor_identifier with SENSOR_ID.get(sensor_identifier)
        if fmt == '<B':  # Special handling for uint8_t
            value = data[offset+4]
        else:
            value = struct.unpack(fmt, data[offset+4:offset+8])[0]
        packet[name] = value
        offset += 8  # Move to the next sensor data field

    packet['crc'] = struct.unpack('<H', data[-2:])[0]
    return packet


def monitor_and_log_serial(log_file, log_time, log_premature = 20, printlog = False):
    print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Logging data to {log_file}. Press Ctrl+C to exit.")
    end_time = -1.
    end_time_premature = time.time() + log_premature
    # Open the serial port and log file
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(log_file, 'w') as log_file:
        buffer = bytearray()
        log_file.write(', '.join(name for name, _ in SENSOR_DATA) + "\n")  # Write header
        start_mature = False
        try:
            while (end_time < 0) or (start_mature and time.time() < end_time) :
                byte = ser.read(1)
                if not byte:
                    continue

                buffer.extend(byte)

                # If a start byte is found and we have a full packet, process it.
                if buffer[-1] == START_BYTE and len(buffer) >= PACKET_SIZE:
                    packet = parse_packet(buffer)
                    if packet:
                        log_entry = ', '.join(f"{packet[name]:.8f}" for name, _ in SENSOR_DATA)
                        if printlog: 
                            print(log_entry)
                        log_file.write(log_entry + "\n")
                        log_file.flush()
                        buffer = bytearray()  # Reset the buffer
                
                if time.time() > end_time_premature and not start_mature:
                    print("Logging time expired, stopping...")
                    print(f" Adjustment complete, start recording for {log_time} sec ")
                    log_file.write(f" Adjustment complete, start recording for {log_time} " + "\n")
                    end_time = time.time() + log_time
                    start_mature = True


                        
                
                # # if I click space, mark in the log file with a line
                # def on_press(key):
                #     if key == Key.space:
                #         
                # with Listener(on_press=on_press) as listener:
                #     listener.join()


                        # print(log_entry, end='\r')
        except KeyboardInterrupt:
            print("\nLogging stopped.")
    
    ser.close()
    log_file.close()





if __name__ == "__main__":
    monitor_and_log_serial(log_file="test.csv", log_time = 100, printlog = True)
