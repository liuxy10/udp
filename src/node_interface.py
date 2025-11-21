
import time
import serial
import numpy as np
import control as ct
import pathlib 
import os, sys

import asciichartpy as acp
from wireless_protocol_library import TcpCommunication, WirelessProtocolLibrary

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import math
# from src.connection.device import *
from src.utils import  popFirstNSamples, set_user_parameters_batch
from src.connection.udp import *
from src.connection.device import *
import time 
import zmq
import select
import termios
import tty
import zmq


def interface(wireless, log_file, in_port = '3333', out_port = '4444',
                                ):
    """ 
    Monitor the wireless device, extract features, communicate via ZMQ, 
    and receive new parameters to set on the device.
    """
    
    # ZMQ Context and Sockets
    context = zmq.Context()
    # Socket to send messages out
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://*:{out_port}")
    print(f"ZMQ Publisher bound to tcp://*:{out_port}")

    # Socket to receive messages
    subscriber = context.socket(zmq.SUB)
    subscriber.connect(f"tcp://localhost:{in_port}")
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "") # Subscribe to all topics
    print(f"ZMQ Subscriber connected to tcp://localhost:{in_port}")

    traj_iter = 0 # iterator of trajectory buffer
    n_traj = 4 # max number of trajectories in the traj buffer
    
    traj_buffer = {
                    "GAIT_SUBPHASE": [[] for _ in range(n_traj)],
                    "LOADCELL": [[] for _ in range(n_traj)],
                    "ACTUATOR_POSITION":[[] for _ in range(n_traj)],
                    "TORQUE_ESTIMATE":[[] for _ in range(n_traj)],
                    }
    # his = {"param": []} # history of current state
    
    # user parameters from firmware 
    params = np.array([get_stance_flexion_level(wireless), get_swing_flexion_angle(wireless), get_toa_torque_level(wireless)])
    param_names = ["stance_flexion_level", "swing_flexion_angle", "toa_torque_level"]

    paused = False

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # Switch terminal to cbreak mode

    
    def check_for_pause():
        nonlocal paused
        if select.select([sys.stdin], [], [], 0)[0]:
            char = sys.stdin.read(1)
            if char == ' ':
                paused = not paused
                print("Paused" if paused else "Resumed")

    
    # Open the serial port and log file
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(log_file, 'w') as log_file:
        print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Press Ctrl+C to exit. Saving data to {log_file}.")
        log_file.write(
            # "TIME_PC" + ',' +# PC received time stamp
            ','.join(name for name, _ in SENSOR_DATA) + # sensor data
            ',' + ','.join(["stance_flexion_level", "swing_flexion_angle", "toa_torque_level"]) + # user parameters
            "\n"
        )  # Write header
        
        buffer = bytearray()
        # try:
        while True:
            check_for_pause()
            if paused:
                time.sleep(0.1)
                continue

            # Check for new parameters from ZMQ
            byte = ser.read(1)
            if not byte:
                continue
            buffer.extend(byte)
            if buffer[-1] == START_BYTE and len(buffer) >= PACKET_SIZE:
                packet = parse_packet(buffer)
                
                if packet and np.all(np.array([v for v in packet.values()]) > -1e6) and np.all(np.array([v for v in packet.values()]) < 1e6): 
                    # add to traj buffer
                    # print(packet.keys())
                    for name in traj_buffer.keys():
                        traj_buffer[name][traj_iter].append(packet[name])
                    
                    # for each gait cycle
                    if (len(traj_buffer["GAIT_SUBPHASE"][traj_iter]) > 50 and
                        np.array(traj_buffer["GAIT_SUBPHASE"][traj_iter])[-1] == 0 and 
                        np.all(np.array(traj_buffer["GAIT_SUBPHASE"][traj_iter])[-10:-1] == 3)):
                        traj_iter += 1
                        if traj_iter >= n_traj:
                            publisher.send_json({"trajectory": traj_buffer})
                            traj_buffer = popFirstNSamples(traj_buffer, 1)
                            for k in traj_buffer.keys():
                                traj_buffer[k].append([])
                            traj_iter -= 1
                    
                    # log data for each packet
                    log_entry = ','.join(f"{packet[name]:.8f}" for name, _ in SENSOR_DATA)
                    log_entry += ',' + ','.join(f"{p}" for p in params)
                    log_file.write(log_entry + "\n")
                    log_file.flush()
                    
                    buffer = bytearray()
                else: 
                    buffer = bytearray()
                
            # check if there are new parameters from ZMQ subscriber (non-blocking)
            try:
                message = subscriber.recv_json(flags=zmq.NOBLOCK)
                if 'params' in message:
                    print(f"RECEIVED the paramer, updated to {message['params']}")
                    new_params = message['params']
                    new_params = np.array([new_params.get(name, params[i]) for i, name in enumerate(param_names)], dtype=int)
                    if not np.array_equal(new_params, params):
                        print(f"Received new params from ZMQ: {new_params}, previous params: {params}")
                        params = new_params
                        set_user_parameters_batch(wireless, params[0], params[1], params[2])
            except zmq.Again:
                pass # No message received


    publisher.close()
    subscriber.close()
    context.term()

    print("ZMQ connections and serial port closed.")
    ser.close()
    print("Interface task completed.")


################## OLD monitor function ##################e


def monitor(wireless, log_file, vis = False, change = [True, True, True]):
    """ Monitor the wireless device and extract features from the data """
    
    traj_iter = 0 # iterator of trajectory buffer
    done = False # flag of end of trial

    n_traj = 4 # max number of trajectories in the traj buffer
    n_init_gait = 10 #  initial gait cycles, where no action updates
    n_action_update= 8 # update the action every n_action_update gait cycles
   

    traj_buffer = {
                     "GAIT_SUBPHASE": [[] for _ in range(n_traj)],
                     "LOADCELL": [[] for _ in range(n_traj)],
                     "ACTUATOR_POSITION":[[] for _ in range(n_traj)],
    }
    his = {"param": [],  # history of current state
           } # indicates if the parameters are at the bound
    
    # user parameters
    params = np.array([get_stance_flexion_level(wireless), get_swing_flexion_angle(wireless), get_toa_torque_level(wireless)])
    change = np.array(change)

    # Open the serial port and log file
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(log_file, 'w') as log_file:
        print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Press Ctrl+C to exit. Saving data to {log_file}.")
        log_file.write(
            "TIME_MILLI" + ',' +# PC received time stamp
            ','.join(name for name, _ in SENSOR_DATA) + # sensor data
            ',' + ','.join(["stance_flexion_level", "swing_flexion_angle", "toa_torque_level"]) + # user parameters
            "\n"
        )  # Write header
        
        buffer = bytearray()
        # try:
        while True:
            byte = ser.read(1)
            if not byte:
                continue
            buffer.extend(byte)
            # If a start byte is found and we have a full packet, process it.
            if buffer[-1] == START_BYTE and len(buffer) >= PACKET_SIZE:
                packet = parse_packet(buffer)
                
                if packet and np.all(np.array([v for v in packet.values()]) > -1e6) and np.all(np.array([v for v in packet.values()]) < 1e6): 
                    log_entry = ','.join(f"{packet[name]:.8f}" for name, _ in SENSOR_DATA)
                    # add to traj buffer
                    for name in traj_buffer.keys():
                        traj_buffer[name][traj_iter].append(packet[name])
                        # print(name, traj_buffer[name][traj_iter], packet[name])
                    # for each gait cycle
                    if (np.array(traj_buffer["GAIT_SUBPHASE"][traj_iter])[-1] == 0 and # the last gait phase is 0
                        np.all(np.array(traj_buffer["GAIT_SUBPHASE"][traj_iter])[-min(10, len(traj_buffer["GAIT_SUBPHASE"][traj_iter])):-1] == 3) and # # the last 10 gait phases are all 3
                        len(traj_buffer["GAIT_SUBPHASE"][traj_iter]) > 50): # actual gait phase change or pseudo
                        print("here")
                        traj_iter += 1
                        
                        # extract features for each gait cycle # TODO: replace by broadcasting func
                        #################### after n_init_gait initial gait cycles, update the action 1 per n_action_update gait cycles ######################
                        if len(his["param"]) > n_init_gait and (len(his["param"])+1) % n_action_update == 0: # only start updating the model after the first few gait cycles
                            prev = np.array([get_stance_flexion_level(wireless), get_swing_flexion_angle(wireless), get_toa_torque_level(wireless)])
                            params = np.array(np.random.uniform(low=[40, 40, 0], high=[75, 75, 100], size=3), dtype = int) # random action for now
                            params[~change] = prev[~change] # control parameter with change = True
                            set_user_parameters_batch(wireless, params[0], params[1], params[2]) # set the user parameters
                        ###################### Evaluation of convergence after  n_init_gait cycles, and every 10 gait cycles after that ######################
                        if len(his["param"]) >= n_init_gait:
                            conv = False # TODO: replace by actual result from RL agent
                            done = conv or len(his["param"]) > 200 # if converged or the number of samples exceeds 100
                        if done:
                            print(f"converged or done at {len(his['param'])} samples, stopping...")
                            break 
                        #################### add the sample to the history and check convergence or done #####################
                        his["param"].append(params) # add the user parameters to the history
                        
                        
                        ###################### reset trajectory buffer ######################
                        # print("reset traj buffer")
                        if traj_iter >= n_traj: # if the trajectory buffer is full, reset it
                            print(f"before his idx: {len(his['param'])}, params: {params}")
                            popFirstNSamples(traj_buffer, 1)  # pop the first sample from each buffer
                            print(f"after his idx: {len(his['param'])}, params: {params}")

                            for k in traj_buffer.keys():
                                traj_buffer[k].append([]) # append a new empty list to the buffer 
                            traj_iter -= 1
                            
                    # log data for each packet
                    if len(his["param"]) > 0: # only after action is taken, to make sure the lqr data is available
                        # log the data to the file
                        log_entry = ','.join(f"{packet[name]:.8f}" for name, _ in SENSOR_DATA) # f"{time_pc:.8f}," + 
                        
                        log_entry += ',' + ','.join(f"{params[k]}" for k in range(len(params)))
                        # print(log_entry)
                        log_file.write(log_entry + "\n")
                        log_file.flush()
                    buffer = bytearray()  # Reset the buffer      
                else: 
                    buffer = bytearray()  # Reset the buffer
            
        

        # except KeyboardInterrupt:
        #     print("\nLogging stopped.")
    
    ser.close()

    print("Feature extraction completed.")


def test_monitor():
    os.system('cls' if os.name == 'nt' else 'clear')
    
    BASE_DIR = pathlib.Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ENV_DIR = BASE_DIR / "env"
    bionics_json_path = ENV_DIR / "bionics.json"
    var_name_json_path = ENV_DIR / "var_names.json"
    DATA_DIR = pathlib.Path("~/Documents/Data/ossur").expanduser()

    wireless = WirelessProtocolLibrary(TcpCommunication(), bionics_json_path) # Time out meaning that the power knee is not connected
    save_folder = DATA_DIR / "adaptive_LQR_0709/test"
    if not save_folder.exists():
        save_folder.mkdir(parents=True, exist_ok=True)
    # test 
    set_stance_flexion_level(wireless, 59)# initial stance flexion level
    set_toa_torque_level(wireless, 50) # ?? --> replaced by swing initiation
    set_swing_flexion_angle(wireless, 50) # target flexion angle
    
    print("DEFAULT initial stance flexion",get_stance_flexion_level(wireless))
    print("DEFAULT max flexion angle", get_swing_flexion_angle(wireless))
    print("DEFAULT swing initiation", get_toa_torque_level(wireless))

    time_stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    # monitor_and_feature_extraction(wireless, x_d = np.array([1., 1., 1.]), vis = True, log_file = save_folder / f"log_{time_stamp}.csv")
    # constraint the activity to be ACTIVITY_FORWARD_PROG
    set_activity(wireless, 1) # set activity to forward progression walking
    # Rerun the main logic to capture output
    monitor(wireless, vis = True, log_file = save_folder / f"log_{time_stamp}.csv")

def test_interface(log_file= "test.csv"):
    
    os.system('cls' if os.name == 'nt' else 'clear')
    BASE_DIR = pathlib.Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ENV_DIR = BASE_DIR / "env"
    bionics_json_path = ENV_DIR / "bionics.json"
    var_name_json_path = ENV_DIR / "var_names.json"
    DATA_DIR = pathlib.Path("~/Documents/Data/ossur").expanduser()

    wireless = WirelessProtocolLibrary(TcpCommunication(), bionics_json_path) # Time out meaning that the power knee is not connected

    # test 
    set_stance_flexion_level(wireless, 59)# initial stance flexion level
    set_toa_torque_level(wireless, 50) # ?? --> replaced by swing initiation
    set_swing_flexion_angle(wireless, 50) # target flexion angle
    
    print("DEFAULT initial stance flexion",get_stance_flexion_level(wireless))
    print("DEFAULT max flexion angle", get_swing_flexion_angle(wireless))
    print("DEFAULT swing initiation", get_toa_torque_level(wireless))

    # monitor_and_feature_extraction(wireless, x_d = np.array([1., 1., 1.]), vis = True, log_file = save_folder / f"log_{time_stamp}.csv")
    # constraint the activity to be ACTIVITY_FORWARD_PROG
    set_activity(wireless, 1) # set activity to forward progression walking
    os.makedirs(DATA_DIR / "OT_11_17", exist_ok=True)
    interface(wireless,log_file= os.path.join(DATA_DIR, "OT_10_31", log_file), in_port="3333", out_port="4444")

def emulate_interface():
    """ Emulate interface function, but not connected to the wireless device """
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind("tcp://*:4444")
    print("ZMQ Publisher bound to tcp://*:4444. Press SPACE to pause/resume.")

    traj_buffer = {
                    "GAIT_SUBPHASE": [[]],
                    "LOADCELL": [[]],
                    "ACTUATOR_POSITION":[[]],
                    "TORQUE_ESTIMATE":[[]],
                    }

    count = 0
    paused = False

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # Switch terminal to cbreak mode

    
    def check_for_pause():
        nonlocal paused
        if select.select([sys.stdin], [], [], 0)[0]:
            char = sys.stdin.read(1)
            if char == ' ':
                paused = not paused
                print("Paused" if paused else "Resumed")

    while True:
        check_for_pause()
        if paused:
            time.sleep(0.1)
            continue

        # Emulate data
        traj_buffer["GAIT_SUBPHASE"].append([0.]*25 + [3.]*25)
        traj_buffer["LOADCELL"].append([0.]*50)
        c1, c2, c3 = 20 + np.random.rand()*5, 20 + np.random.rand()*5, 50 + np.random.rand()*10
        traj_buffer["ACTUATOR_POSITION"].append([math.sin(i/25 * np.pi)* c1 + c2 + np.random.rand()*5  for i in range(25)] \
                                                + [math.sin(i/25 * np.pi*1.2)* c3 + c2 + np.random.rand()*5 for i in range(25)])
        traj_buffer["TORQUE_ESTIMATE"].append([0.]*50)

        if len(traj_buffer["GAIT_SUBPHASE"]) > 4:
            for k in traj_buffer.keys():
                traj_buffer[k].pop(0)
        # Send data
        publisher.send_json({"trajectory": traj_buffer})
    
        print("count:", count, f"sent trajectory data {list(traj_buffer.keys())}")
        count += 1
        time.sleep(0.1)  # Emulate a delay

if __name__ == "__main__": # example usage
    # test_monitor()
    # test_interface(log_file=f"trial_{time.strftime('%Y%m%d_%H%M%S', time.localtime())}")
    emulate_interface()