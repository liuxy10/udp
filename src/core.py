
import time
import serial
import numpy as np
import control as ct
import pathlib 
import os, sys

import asciichartpy as acp
from wireless_protocol_library import TcpCommunication, WirelessProtocolLibrary

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))
from connection.device import *
from connection.udp import *
"""copy from adaptive_LQR_master.py, TODO refactored to be the core node of the system"""

def addSample(his, state, action, done, x_d):
    """ Add a sample to the history """
    his["current_state"].append(state.copy())
    his["action"].append(action.copy())
    his["stage_cost"].append(np.linalg.norm(state - x_d))
    his["done"].append(done)
    return his

def popFirstNSamples(dict, n):
    """ Pop the first n samples from the history """
    first_key = next(iter(dict)) # get the first key
    n = min(n, len(dict[first_key]))
    return {k: v[n:] for k, v in dict.items()}

def feature_selection_from_mean_traj(traj_buffer, vis = False): # for multiple gait cycles
    """ Extract features from the trajectory buffer """
    mean = {k: [] for k in traj_buffer.keys()}  # initialize mean dictionary
    i = 0
    for k in traj_buffer.keys():
        for traj in traj_buffer[k]:
            if len(traj) > 0: # Interpolate to handle missing or unevenly sampled data
                traj = np.array(traj)
                x = np.linspace(0, 1, len(traj))
                x_uniform = np.linspace(0, 1, 100)
                traj_interp = np.interp(x_uniform, x, traj)
                mean[k].append(traj_interp)
        mean[k] = np.mean(np.array(mean[k]), axis=0)  # take the mean of the trajectories
        assert len(mean[k]) == 100, f"Mean trajectory for {k} is not of length 100, instead, got {len(mean[k])}"
    return feature_selection_per_gait(mean, vis)  # extract features from the mean trajectory
    
def feature_selection_per_gait(dat, vis): # for one gait cycle
    dat = smooth_data(dat, window_size=5, names = ["LOADCELL", "ACTUATOR_POSITION"]) # smooth the data
    if vis: 
        vis_gait_cycle_terminal(dat)
    # print(f"Extracting features from gait cycle...")
    # Find the timing (index) of the transition from 0 to 1 in "GAIT_SUBPHASE"
    gait_subphase = np.array(dat["GAIT_SUBPHASE"])
    transition_idx = np.where((gait_subphase[:-1] == 0))[0][-1] if sum(gait_subphase[:-1] == 0) > 0 else -1 
    st_sw_phase = transition_idx / len(gait_subphase )   
    # print(f"st_sw_phase: {st_sw_phase}")
    # duration of brake time (subphase 4)
    brake_indices = np.where(gait_subphase  == 4)[0]
    if len(brake_indices) > 0:
        # Assuming 100 Hz sampling rate, so duration = count / 100
        brake_time = len(brake_indices) / len(gait_subphase )   
    else:
        brake_time = 0

    return (st_sw_phase, # stance to swing phase ratio
            brake_time, # brake time ratio
            np.argmin(np.array(dat["ACTUATOR_POSITION"])[:transition_idx]) / len(dat["ACTUATOR_POSITION"]) if transition_idx > 0 else 0)  # min actuator position phase in st
    
def set_user_parameters(wireless, action):
    """ Set the user parameters based on the action """
    action = np.array(action, dtype = int)
    while True: 
        try: 
            c1_ = get_stance_flexion_level(wireless) # get the current stance flexion level
            c2_ = get_swing_flexion_angle(wireless) # get the current swing flexion angle
            c3_= get_toa_torque_level(wireless)
            break
        except Exception as e:
            print(f"Error getting user parameters: {e}")
            time.sleep(0.1)
    c1 = min(max(c1_ + action[0],40), 75) # stance flexion level
    c2 = min(max(c2_ + action[1], 0), 100)
    c3 = min(max(c3_ + action[2], 0), 100) # swing flexion angle
    while True: 
        try:   
            set_stance_flexion_level(wireless, c1)# initial stance flexion level
            set_swing_flexion_angle(wireless, c2) # target flexion angle
            set_toa_torque_level(wireless, c3) # ?? --> replaced by swing initiation

            if get_stance_flexion_level(wireless) == c1 and get_swing_flexion_angle(wireless) == c2 and get_toa_torque_level(wireless) == c3:
                print("successfully update user parameters from ", np.array([c1_, c2_, c3_]), " to ", np.array([c1, c2, c3]))
                os.system("afplay /System/Library/Sounds/Basso.aiff") # play a sound to indicate update of user parameters, "Basso", "Blow", "Bottle", "Frog", "Funk", "Glass", "Hero", "Morse", "Ping", "Pop", "Purr", "Sosumi", "Submarine", and "Tink"
                break
        except Exception as e:
            print(f"Error setting user parameters: {e}")
            time.sleep(0.1)

    return np.array([c1 - c1_, c2 - c2_, c3 - c3_]), {"stance_flexion_level": c1, "swing_flexion_angle": c2, "toa_torque_level": c3}  # return the change in parameters

def vis_gait_cycle_terminal(traj_buffer):
    # clear the terminal 
    # os.system('cls' if os.name == 'nt' else 'clear')
    # Print higher resolution (more rows) for each plot
    print("gait subphase")
    print(acp.plot(np.array(traj_buffer["GAIT_SUBPHASE"]), {'height': 10, 'format': '{:5.2f}'}))
    print("load cell")
    print(acp.plot(np.array(traj_buffer["LOADCELL"]), {'height': 10, 'format': '{:5.2f}'}))
    print("knee angle")
    print(acp.plot(np.array(traj_buffer["ACTUATOR_POSITION"]), {'height': 10, 'format': '{:5.2f}'}))

def smooth_data(data, window_size=5, names = ["LOADCELL", "ACTUATOR_POSITION"]):
    """ Smooth the data using a simple moving average"""
    for name in names:
        if name in data.keys():
            data[name] = np.convolve(data[name], np.ones(window_size)/window_size, mode='valid')
    return data

def monitor_and_feature_extraction(wireless, log_file,  x_d = np.array([0,0,0]), vis = False):
    
    # set_activity(wireless, 0) # set activity to 0 (level ground walking )
    print("initializing feature extraction...")
    his = {"current_state": [],  # history of current state
           "action": [],  # history of actions
           "stage_cost": [], # history of stage cost
           "done":[]} # indicates if the parameters are at the bound
    traj_iter, n_traj = 0, 4
    traj_buffer = {"GAIT_PHASE": [[] for _ in range(n_traj)],
                     "GAIT_SUBPHASE": [[] for _ in range(n_traj)],
                     "LOADCELL": [[] for _ in range(n_traj)],
                     "ACTUATOR_POSITION":[[] for _ in range(n_traj)],
    }
    
    n_action_update= 4 # update the action every n_action_update gait cycles
    # user parameters
    params = {"stance_flexion_level": get_stance_flexion_level(wireless), "swing_flexion_angle": get_swing_flexion_angle(wireless), "toa_torque_level": get_toa_torque_level(wireless)} 
    state_buffer = []
    # define the dynamics to be x (k+1) = x(k) + A u(k) + w(k). 
    A_est = np.random.rand(3, 3)  # Example A matrix for state transition
    Qs = np.eye(3) * 0.1 # State cost matrix
    Rs = np.eye(3) * 0.01 # Action cost matrix
    # Open the serial port and log file
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(log_file, 'w') as log_file:
        print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Press Ctrl+C to exit. Saving data to {log_file}.")
        log_file.write(
            ','.join(name for name, _ in SENSOR_DATA) +
            ',' + ','.join(["stance_flexion_level", "swing_flexion_angle", "toa_torque_level"]) +
            ',' + ','.join(["st_sw_phase", "brake_time","min_knee_position_phase_st"]) +
            ',' + ','.join(["stage_cost", "done"]) +
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
                # print('buffer size', len(buffer))
                packet = parse_packet(buffer)
                if packet and np.all(np.array([v for v in packet.values()]) > -1e6) and np.all(np.array([v for v in packet.values()]) < 1e6): 
                    # check if the traj buffer is full
                    # add to traj buffer
                    for name in traj_buffer.keys():
                        traj_buffer[name][traj_iter].append(packet[name])
                    
                    # for each gait cycle
                    if (np.array(traj_buffer["GAIT_PHASE"][traj_iter])[-1] == 0 and # the last gait phase is 0
                        np.all(np.array(traj_buffer["GAIT_PHASE"][traj_iter])[-min(10, len(traj_buffer["GAIT_SUBPHASE"][traj_iter])):-1] )== 1 and # # the last 10 gait phases are all 1
                        len(traj_buffer["GAIT_PHASE"][traj_iter]) > 50): # actual gait phase change or pseudo
                        traj_iter += 1
                        # extract features for each gait cycle
                        st_sw_phase, brake_time, min_knee_position_phase_st = feature_selection_from_mean_traj(traj_buffer, vis = vis)
                        state = np.array([st_sw_phase, brake_time, min_knee_position_phase_st])
                        #################### update the action after 10 gait cycles: 1 per n_action_updategait cycles ######################
                        if len(his["current_state"]) > 10 and (len(his["current_state"])+1) % n_action_update == 0: # only start updating the model after the first few gait cycles
                            try: 
                                K_est, _,_ = ct.lqr(np.eye(3),A_est, Qs, Rs)
                            except:
                                print("LQR failed, using random K_est")
                                K_est = np.random.rand(3,3)
                            action = - 2 * np.inner(state, K_est) # compute the optimal action with the estimated system matrix
                            action_taken, params = set_user_parameters(wireless, action) # set the user parameters
                        else:
                            action = np.zeros(3)
                            action_taken = np.zeros(3) # no action taken in the first few gait cycles    
                        ###################### Evaluation of convergence after 30 initial gait cycles, and every 10 gait cycles after that ######################
                        if len(his["action"]) >= 10:
                            conv = np.max(np.abs(np.array(his["current_state"])[-min(5, len(his["current_state"])-1):] - x_d)) < 0.1 # the last 5 samples are within the bound
                            done = conv or len(his["current_state"]) > 200 # if converged or the number of samples exceeds 100
                        else:
                            conv = False
                            done = False
                        # print(f"his {his}")
                        his = addSample(his, state, action_taken, done, x_d)
                        print(f"his idx: {len(his['action'])}, state: {state}, action: {action}, action taken: {action_taken}, done: {done}")
                        if conv: # if converge
                            print(f"Converged at index {len(his['action'])}, state: {state}, action: {action}")
                            os.system("afplay /System/Library/Sounds/Glass.aiff") # play a sound to indicate convergence
                            break # exit the WHOLE loop
                    
                        if done: # successfully converge or numbers of steps reached
                            print(f"Done at index {len(his['action'])}, state: {state}, action: {action}")
                            os.system("afplay /System/Library/Sounds/Pop.aiff") # play
                            break

                        if (len(his["action"]) + 1) % 15 == 0: # if not converge but done 
                            os.system("afplay /System/Library/Sounds/Funk.aiff") # play a sound to indicate A updating
                            # use least square on the F-norm of 
                            state_diff = np.diff(np.array(his["current_state"]),1, axis = 0)
                            assert state_diff.shape[0] == len(his["action"]) - 1, "State error stack length does not match action length, instead, got {} and {}".format(state_diff.shape[0], len(his["action"]) - 1)
                            assert state_diff.shape[1] == 3, "State error stack shape is not correct, instead, got {}".format(state_diff.shape)
                            A_est = np.linalg.lstsq(state_diff, np.array(his["action"])[:-1], rcond=None)[0]  # Estimate the system matrix A
                            print(f"Estimated A matrix: \n {A_est}, and collect another few samples")
                            # his =  popFirstNSamples(his, 10)  # pop the first 10 samples

                        # reset a new trajectory buffer
                        # print("reset traj buffer")
                        if traj_iter >= n_traj: # if the trajectory buffer is full, reset it
                            popFirstNSamples(traj_buffer, 1)
                            for k in traj_buffer.keys():
                                  # pop the first sample from each buffer
                                traj_buffer[k].append([packet[k]])
                            traj_iter -= 1
                            

                    if len(his["action"]) > 0: 
                        # log the data to the file
                        log_entry = ','.join(f"{packet[name]:.8f}" for name, _ in SENSOR_DATA)
                        log_entry += ',' + ','.join(f"{params[k]}" for k in ["stance_flexion_level", "swing_flexion_angle", "toa_torque_level"])
                        log_entry += ',' + ','.join(f"{his['current_state'][-1][i]}" for i in range(len(state)))  # add the state
                        log_entry += ',' + ','.join(f"{his[k][-1]:.8f}" for k in ["stage_cost", "done"])
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


def adhoc_feature_extraction(log_file, vis = False):
    """ Ad-hoc feature extraction for each gait cycles """
    
    

if __name__ == "__main__": # example usage
    os.system('cls' if os.name == 'nt' else 'clear')
    
    BASE_DIR = pathlib.Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ENV_DIR = BASE_DIR / "env"
    bionics_json_path = ENV_DIR / "bionics.json"
    var_name_json_path = ENV_DIR / "var_names.json"
    DATA_DIR = pathlib.Path("~/Documents/Data/ossur").expanduser()

    wireless = WirelessProtocolLibrary(TcpCommunication(), bionics_json_path) # Time out meaning that the power knee is not connected
    save_folder = DATA_DIR / "adaptive_LQR_0709/004"
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
    monitor_and_feature_extraction(wireless, x_d = np.array([1., 1., 1.]), vis = True, log_file = save_folder / f"log_{time_stamp}.csv")
   