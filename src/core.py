
import time
import serial
import numpy as np
import control as ct
import pathlib 
import os, sys

import asciichartpy as acp
from wireless_protocol_library import TcpCommunication, WirelessProtocolLibrary

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))
# from src.connection.device import *
from src.utils import  popFirstNSamples, set_user_parameters_batch, feature_selection_from_mean_traj
from src.connection.udp import *
from src.connection.device import *
import time 



## TODO: post hoc analysis of adaptive LQR algorithm
def monitor(wireless, log_file, vis = False, change = [True, True, True]):
    """ Monitor the wireless device and extract features from the data """
    
    traj_iter = 0 # iterator of trajectory buffer
    done = False # flag of end of trial

    n_traj = 4 # max number of trajectories in the traj buffer
    n_init_gait = 10 #  initial gait cycles, where no action updates
    n_action_update= 10 # update the action every n_action_update gait cycles
   

    traj_buffer = {"GAIT_PHASE": [[] for _ in range(n_traj)],
                     "GAIT_SUBPHASE": [[] for _ in range(n_traj)],
                     "LOADCELL": [[] for _ in range(n_traj)],
                     "ACTUATOR_POSITION":[[] for _ in range(n_traj)],
    }
    his = {"param": [],  # history of current state
           } # indicates if the parameters are at the bound
    
    # user parameters
    params = np.array([get_stance_flexion_level(wireless), get_swing_flexion_angle(wireless), get_toa_torque_level(wireless)])
    change = np.array(change)

    start_time = time.time()
    # Open the serial port and log file
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(log_file, 'w') as log_file:
        print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Press Ctrl+C to exit. Saving data to {log_file}.")
        log_file.write(
            "TIME_PC" + # PC received time stamp
            ',' +','.join(name for name, _ in SENSOR_DATA) + # sensor data
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
                    # check if the traj buffer is full
                    time_pc = time.time() - start_time 
                    # add to traj buffer
                    for name in traj_buffer.keys():
                        traj_buffer[name][traj_iter].append(packet[name])
                    
                    # for each gait cycle
                    if (np.array(traj_buffer["GAIT_PHASE"][traj_iter])[-1] == 0 and # the last gait phase is 0
                        np.all(np.array(traj_buffer["GAIT_PHASE"][traj_iter])[-min(10, len(traj_buffer["GAIT_SUBPHASE"][traj_iter])):-1] )== 1 and # # the last 10 gait phases are all 1
                        len(traj_buffer["GAIT_PHASE"][traj_iter]) > 50): # actual gait phase change or pseudo
                        
                        traj_iter += 1
                        _,_,_ = feature_selection_from_mean_traj(traj_buffer, vis = vis)
                        
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
                        print(f"his idx: {len(his['param'])}, params: {params}")
                        

                        ###################### reset trajectory buffer ######################
                        # print("reset traj buffer")
                        if traj_iter >= n_traj: # if the trajectory buffer is full, reset it
                            popFirstNSamples(traj_buffer, 1)  # pop the first sample from each buffer
                            for k in traj_buffer.keys():
                                traj_buffer[k].append([]) # append a new empty list to the buffer 
                            traj_iter -= 1
                            
                    # log data for each packet
                    if len(his["param"]) > 0: # only after action is taken, to make sure the lqr data is available
                        # log the data to the file
                        log_entry = f"{time_pc:.8f}," + ','.join(f"{packet[name]:.8f}" for name, _ in SENSOR_DATA)
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



    

if __name__ == "__main__": # example usage
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
   