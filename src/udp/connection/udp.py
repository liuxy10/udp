import serial
import struct
import time
import pathlib
import os

import numpy as np

START_BYTE = 0xAA
PACKET_SIZE = 200
BAUD_RATE = 460800
SERIAL_PORT = '/dev/cu.usbserial-ABSCHWQ0' # ls /dev/cu.usb* # 'COM5' #

BASE_DIR = pathlib.Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LOG_FILE = BASE_DIR / "logs" / f"{time.strftime('%Y-%m-%d_%H-%M-%S')}_udp_core.csv"
# os.makedirs(LOG_FILE, exist_ok=True)

# Define sensor data format: (name, struct-format)
SENSOR_DATA = [
    ('TIME_MILLI', '<f'),
    ('ACTIVITY', '<B'),  # ('ACTIVITY', '<I'),
    ('GAIT_SUBPHASE', '<B'),  # Special handling for uint8_t:     SUBPHASE_FORCE_REJECTION, SUBPHASE_TOE_OFF_ASSIST, SUBPHASE_BUMPER_AVOIDANCE, SUBPHASE_FORCE_FOLLOWING, SUBPHASE_BRAKE
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
    ('SHANK_ANGLE', '<f'),
]

# Mapping sensor identifier (4 bytes) to sensor names
SENSOR_ID = {
    0x44035D71: 'TIME_MILLI',  # BIONICS_VAR_BASE_TIME_SYSTEM_ON
    0x24111110: 'ACTIVITY',      # BIONICS_VAR_BASE_GAIT_PHASE
    0x2101C610: 'GAIT_SUBPHASE',   # BIONICS_VAR_BASE_GAIT_SUBPHASE
    0x44010770: 'ACTUATOR_POSITION',  # BIONICS_VAR_KNEE_ACTUATOR_POSITION
    0x44010171: 'TORQUE_ESTIMATE',    # BIONICS_VAR_POWER_KNEE_TORQUE_EST
    0x44011471: 'ACTUATOR_SETPOINT',  # BIONICS_VAR_POWER_KNEE_ACTUATOR_SETPOINT
    0x34118830: 'LOADCELL',           # BIONICS_VAR_LEG_GROUND_REACTION_FORCE
    0x4401AC10: 'LINEAR_ACC_X_LOCAL', # BIONICS_VAR_BASE_LIN_ACC_X_LOCAL
    0x4401AD10: 'LINEAR_ACC_Y_LOCAL', # BIONICS_VAR_BASE_LIN_ACC_Y_LOCAL
    0x4401AE10: 'LINEAR_ACC_Z_LOCAL', # BIONICS_VAR_BASE_LIN_ACC_Z_LOCAL
    0x4401B210: 'GRAVITY_VECTOR_X',   # BIONICS_VAR_BASE_GRAV_VECT_X
    0x4401B310: 'GRAVITY_VECTOR_Y',   # BIONICS_VAR_BASE_GRAV_VECT_Y
    0x4401B410: 'GRAVITY_VECTOR_Z',   # BIONICS_VAR_BASE_GRAV_VECT_Z
    0x44118930: 'SHANK_ANGLE',         # BIONICS_VAR_KNEE_JOINT_ANGLE
}

MIN_MAX_BOUNDS = {
    # find the min max value from bionics.json
    'TIME_MILLI': (0, 1e7),
    'ACTIVITY': (0, 10),  # Example range for activity
    'GAIT_SUBPHASE': (0, 6),  # Example range for gait subphase
    'ACTUATOR_POSITION': (0, 100),  # Example range for actuator
    'TORQUE_ESTIMATE': (-100, 100),  # Example range for torque estimate
    'ACTUATOR_SETPOINT': (-100, 100),  # Example range for actuator setpoint
    'LOADCELL': (0, 300),  # Example range for load cell
    'LINEAR_ACC_X_LOCAL': (-50, 50),  # Example range for linear acceleration
    'LINEAR_ACC_Y_LOCAL': (-50, 50),  # Example range for linear acceleration
    'LINEAR_ACC_Z_LOCAL': (-50, 50),  # Example range for linear acceleration
    'GRAVITY_VECTOR_X': (-10, 10),  # Example range for gravity vector
    'GRAVITY_VECTOR_Y': (-10, 10),  # Example range for gravity vector
    'GRAVITY_VECTOR_Z': (-10, 10),  # Example range for gravity vector
    'SHANK_ANGLE': (-180, 180),  # Example range for shank angle
}

def any_entry_out_of_bound(packet):
    for name, _ in SENSOR_DATA:
        if name in packet:
            value = packet[name]
            min_val, max_val = MIN_MAX_BOUNDS.get(name, (None, None))
            if min_val is not None and max_val is not None:
                if not (min_val <= value <= max_val):
                    print(f"Value out of bounds for {name}: {value} not in ({min_val}, {max_val})")
                    return True
    return False


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
        elif fmt == '<h':  # Special handling for int16_t
            value = struct.unpack(fmt, data[offset+4:offset+6])[0]
        elif fmt == '<f': # Special handling for float
            value = struct.unpack(fmt, data[offset+4:offset+8])[0]
        else: 
            print("Unsupported format:", fmt)
        # if abs(value) > 1e7 and name in SENSOR_ID.values(): #['TIME_MILLI', 'GRAVITY_VECTOR_X', 'GRAVITY_VECTOR_Y', 'GRAVITY_VECTOR_Z']:  # Check for invalid values
        min_val, max_val = MIN_MAX_BOUNDS.get(name, (None, None))
        if min_val is not None and max_val is not None:
            if not (min_val <= value <= max_val):
                print(f"Value out of bounds for {name}: {value} not in ({min_val}, {max_val}) Skipping this whole packet.")
                return None
        packet[name] = value
        offset += 8  # Move to the next sensor data field

    packet['crc'] = struct.unpack('<H', data[-2:])[0]
    return packet


def monitor_and_log_serial(log_file, log_time, log_premature = 20, printlog = False):
    # print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Logging data to {log_file}. Press Ctrl+C to exit.")
    # end_time = -1.
    start_time = time.time()
    end_time = start_time + log_time
    end_time_premature = time.time() + log_premature
    # Open the serial port and log file
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(log_file, 'w') as log_file:
        print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Logging data to {log_file}. Press Ctrl+C to exit.")
        buffer = bytearray()
        log_file.write(','.join(name for name, _ in SENSOR_DATA) + "\n")  # Write header
        start_mature = True
        # try:
        while True:
            while (end_time < 0) or (start_mature and time.time() < end_time) :
            # while True:
                byte = ser.read(1)
                if not byte:
                    continue
                buffer.extend(byte)

                # If a start byte is found and we have a full packet, process it.
                if buffer[-1] == START_BYTE and len(buffer) >= PACKET_SIZE:
                    # print('buffer size', len(buffer))
                    packet = parse_packet(buffer)
                    if packet:
                        # if printlog: 
                            # diff = get_shank_angle_from_gravity_vec_degree(packet["GRAVITY_VECTOR_X"], packet["GRAVITY_VECTOR_Y"]) + packet["SHANK_ANGLE"]
                            # # print("calculated shank angle = ", cal)
                            # # print(f"diff in shank = {diff:.8f}")
                            # # print(get_shank_angle_from_gravity_vec_degree(packet["GRAVITY_VECTOR_X"], packet["GRAVITY_VECTOR_Y"]), - packet["SHANK_ANGLE"])
                        
                        # log the data
                        log_entry = ','.join(f"{packet[name]:.8f}" for name, _ in SENSOR_DATA)
                        print(log_entry)
                        log_file.write(log_entry + "\n")
                        log_file.flush()
                        buffer = bytearray()  # Reset the buffer
                    else:
                        buffer = bytearray()  # Reset the buffer
                
                if time.time() > end_time_premature and not start_mature:
                    print("Logging time expired, stopping...")
                    print(f" Adjustment complete, start recording for {log_time} sec ")
                    log_file.write(f" Adjustment complete, start recording for {log_time} " + "\n")
                    end_time = time.time() + log_time
                    start_mature = True

        # except KeyboardInterrupt:
        #     print("\nLogging stopped.")
    
    ser.close()
    log_file.close()

def get_shank_angle_from_gravity_vec_degree(gx, gy):
    return - np.arcsin(gx/np.sqrt(gx**2 + gy**2)) * 180/np.pi

    
if __name__ == "__main__":
    log_time = 10
    monitor_and_log_serial(LOG_FILE, log_time, printlog = True)
#     # plot_data()


