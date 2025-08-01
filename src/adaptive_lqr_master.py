
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
from src.utils import addSample, popFirstNSamples, feature_selection_from_mean_traj, set_user_parameters_action
from src.connection.udp import *
from src.connection.device import *
import time 

def addSample(his, state, action, done, x_d):
    """ Add a sample to the history """
    his["current_state"].append(state.copy())
    his["action"].append(action.copy())
    his["stage_cost"].append(np.linalg.norm(state - x_d))
    his["done"].append(done)
    return his

## TODO: post hoc analysis of adaptive LQR algorithm
def monitor_and_feature_extraction(wireless, log_file,  x_d = np.array([0,0,0]), vis = False):
    """ Monitor the wireless device and extract features from the data """
    traj_iter = 0 # iterator of trajectory buffer
    done = False # flag of end of trial
    n_traj = 4 # max number of trajectories in the traj buffer
    
    n_init_gait = 10 #  initial gait cycles, where no action updates
    n_action_update= 4 # update the action every n_action_update gait cycles
    n_est_update = 15 # update the system matrix A every n_est_update samples
    # define the dynamics to be x (k+1) = x(k) + A u(k) + w(k). 
    A_est = np.random.rand(3, 3)  # Example A matrix for state transition
    Qs = np.eye(3) * 0.1 # State cost matrix
    Rs = np.eye(3) * 0.01 # Action cost matrix
    
    print("initializing feature extraction...")
    his = {"current_state": [],  # history of current state
           "action": [],  # history of actions
           "stage_cost": [], # history of stage cost
           "done":[]} # indicates if the parameters are at the bound

    traj_buffer = {"GAIT_PHASE": [[] for _ in range(n_traj)],
                     "GAIT_SUBPHASE": [[] for _ in range(n_traj)],
                     "LOADCELL": [[] for _ in range(n_traj)],
                     "ACTUATOR_POSITION":[[] for _ in range(n_traj)],
    }
    
    # user parameters
    params = {"stance_flexion_level": get_stance_flexion_level(wireless), "swing_flexion_angle": get_swing_flexion_angle(wireless), "toa_torque_level": get_toa_torque_level(wireless)} 
    state_buffer = []

    start_time = time.time()
    # Open the serial port and log file
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(log_file, 'w') as log_file:
        print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Press Ctrl+C to exit. Saving data to {log_file}.")
        log_file.write(
            ','.join(name for name, _ in SENSOR_DATA) + # sensor data
            ',' + ','.join(["stance_flexion_level", "swing_flexion_angle", "toa_torque_level"]) + # user parameters
            ',' + ','.join(["st_sw_phase", "brake_time","min_knee_position_phase_st"]) + # features
            ',' + ','.join(["stage_cost", "done"]) + # stage cost and done flag
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
                    packet["TIME_PC"] = time.time() - start_time 
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
                        #################### after n_init_gait initial gait cycles, update the action 1 per n_action_update gait cycles ######################
                        if len(his["current_state"]) > n_init_gait and (len(his["current_state"])+1) % n_action_update == 0: # only start updating the model after the first few gait cycles
                            try: 
                                K_est, _,_ = ct.lqr(np.eye(3),A_est, Qs, Rs)
                            except:
                                print("LQR failed, using random K_est")
                                K_est = np.random.rand(3,3)
                            action = - 2 * np.inner(state, K_est) # compute the optimal action with the estimated system matrix
                            action_taken, params = set_user_parameters_action(wireless, action) # set the user parameters
                        else:
                            action = np.zeros(3)
                            action_taken = np.zeros(3) # no action taken in the first few gait cycles    
                        ###################### Evaluation of convergence after  n_init_gait cycles, and every 10 gait cycles after that ######################
                        if len(his["current_state"]) >= n_init_gait:
                            conv = np.max(np.abs(np.array(his["current_state"])[-min(5, len(his["current_state"])-1):] - x_d)) < 0.1 # the last 5 samples are within the bound
                            done = conv or len(his["current_state"]) > 200 # if converged or the number of samples exceeds 100
                        else:
                            conv = False
                            done = False
                        #################### add the sample to the history and check convergence or done #####################
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
                        ####################### A update every n_est_update samples if not converged ########################
                        if (len(his["action"]) + 1) % n_est_update == 0: # if not converge but done 
                            os.system("afplay /System/Library/Sounds/Funk.aiff") # play a sound to indicate A updating
                            # use least square on the F-norm of 
                            state_diff = np.diff(np.array(his["current_state"]),1, axis = 0)
                            assert state_diff.shape[0] == len(his["action"]) - 1, "State error stack length does not match action length, instead, got {} and {}".format(state_diff.shape[0], len(his["action"]) - 1)
                            assert state_diff.shape[1] == 3, "State error stack shape is not correct, instead, got {}".format(state_diff.shape)
                            A_est = np.linalg.lstsq(state_diff, np.array(his["action"])[:-1], rcond=None)[0]  # Estimate the system matrix A
                            print(f"Estimated A matrix: \n {A_est}, and collect another few samples")
                            # his =  popFirstNSamples(his, 10)  # pop the first 10 samples

                        ###################### reset trajectory buffer ######################
                        # print("reset traj buffer")
                        if traj_iter >= n_traj: # if the trajectory buffer is full, reset it
                            popFirstNSamples(traj_buffer, 1)  # pop the first sample from each buffer
                            for k in traj_buffer.keys():
                                traj_buffer[k].append([]) # append a new empty list to the buffer 
                            traj_iter -= 1
                            
                    # log data for each packet
                    if len(his["action"]) > 0: # only after action is taken, to make sure the lqr data is available
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
    monitor_and_feature_extraction(wireless, x_d = np.array([1., 1., 1.]), vis = True, log_file = save_folder / f"log_{time_stamp}.csv")
   