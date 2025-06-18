
from udp_core import * 
import time
import serial
import numpy as np
import control as ct
from wireless_protocol_library import TcpCommunication, WirelessProtocolLibrary
from CommonTestFunctions import set_stance_flexion_level, set_toa_torque_level, set_swing_flexion_angle, set_activity, get_stance_flexion_level, get_toa_torque_level, get_swing_flexion_angle
import pathlib 
import os 
import asciichartpy as acp

def addSample(his, state, action, done, x_d):
    """ Add a sample to the history """
    his["current_state"].append(state.copy())
    his["action"].append(action.copy())
    his["stage_cost"].append(np.linalg.norm(state - x_d))
    his["done"].append(done)
    return his

def popFirstNSamples(his, n):
    """ Pop the first n samples from the history """
    n = min(n, len(his["current_state"]))
    return {k: v[n:] for k, v in his.items()}

def feature_selection_per_gait(dat): # for one gait cycle
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
                break
        except Exception as e:
            print(f"Error setting user parameters: {e}")
            time.sleep(0.1)

    return np.array([c1 - c1_, c2 - c2_, c3 - c3_])  # return the change in parameters

def vis_gait_cycle_terminal(his, traj_buffer):
    # clear the terminal 
    os.system('cls' if os.name == 'nt' else 'clear')
    print(f"gait cycle # {len(his['action'])}, traj buffer size: {len(traj_buffer['GAIT_PHASE'])}")
    print("gait subphase")
    print(acp.plot(np.array(traj_buffer["GAIT_SUBPHASE"]), {'height': 5, 'format': '{:5.2f}'}) )
    print("load cell")
    print(acp.plot(np.array(traj_buffer["LOADCELL"]), {'height': 5, 'format': '{:5.2f}'}) )
    print("knee angle")
    print(acp.plot(np.array(traj_buffer["ACTUATOR_POSITION"]), {'height': 5, 'format': '{:5.2f}'}) )


def monitor_and_feature_extraction(wireless, x_d = np.array([0,0,0]), vis = False):
    # set_activity(wireless, 0) # set activity to 0 (level ground walking )
    print("initializing feature extraction...")
    his = {"current_state": [],  # history of current state
           "action": [],  # history of actions
           "stage_cost": [], # history of stage cost
           "done":[]} # indicates if the parameters are at the bound
    
    traj_buffer = {"GAIT_PHASE": [],
                     "GAIT_SUBPHASE": [],
                     "LOADCELL": [],
                     "ACTUATOR_POSITION":[]
    }

    A_est = np.random.rand(3, 3)  # Example A matrix for state transition
    Qs = np.eye(3) * 0.1 # State cost matrix
    Rs = np.eye(3) * 0.01 # Action cost matrix
    # Open the serial port and log file
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud. Press Ctrl+C to exit.")
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
                        traj_buffer[name].append(packet[name])

                    if np.array(traj_buffer["GAIT_PHASE"])[-1] == 0 and len(traj_buffer["GAIT_PHASE"]) > 30: # actual gait phase change or pseudo
                    # if len(traj_buffer["GAIT_PHASE"]) > 100: # pseudo TODO: switch to actual 
                        if vis: 
                            vis_gait_cycle_terminal(his, traj_buffer)
                        # print(f"traj buffer: {traj_buffer}")
                        # extract features
                        st_sw_phase, brake_time, min_knee_position_phase_st = feature_selection_per_gait(traj_buffer)
                        state = np.array([st_sw_phase, brake_time, min_knee_position_phase_st])
                        try: 
                            K_est, _,_ = ct.lqr(np.eye(3),A_est, Qs, Rs)
                        except:
                            K_est = np.random.rand(3,3)
                        action = - 2 * np.inner(state, K_est) # compute the optimal action with the estimated system matrix
                        action_taken = set_user_parameters(wireless, action) # set the user parameters
                        
                    
                        ###################################################
                        if len(his["action"]) >= 10:
                            conv = np.max(np.abs(np.array(his["current_state"])[-min(5, len(his["current_state"])-1):] - x_d)) < 0.1 # the last 5 samples are within the bound
                            done = conv or len(his["current_state"]) > 30 # if converged or the history is 
                        else:
                            conv = False
                            done = False
                        # print(f"his {his}")
                        his = addSample(his, state, action_taken, done, x_d)
                        print(f"his idx: {len(his['action'])}, state: {state}, action: {action}, action taken: {action_taken}, done: {done}")
                        if conv: # if converge
                            print(f"Converged at index {len(his['action'])}, state: {state}, action: {action}")
                            break # exit the WHOLE loop
                           
                        elif done: # if not converge but done
                            A_est = np.linalg.lstsq(np.array(his["current_state"]), np.array(his["action"]), rcond=None)[0]  # Estimate the system matrix A
                            print(f"Estimated A matrix: \n {A_est}, and collect another few samples")
                            his =  popFirstNSamples(his, 10)  # pop the first 10 samples


                        # reset a new trajectory buffer
                        # print("reset traj buffer")
                        for k in traj_buffer.keys():
                            traj_buffer[k] = [packet[name]] 

                    buffer = bytearray()  # Reset the buffer      
                else: 
                    buffer = bytearray()  # Reset the buffer
            
        

        # except KeyboardInterrupt:
        #     print("\nLogging stopped.")
    
    ser.close()

    print("Feature extraction completed.")



if __name__ == "__main__":

    ROOT = pathlib.Path(os.path.dirname(os.path.abspath(__file__)))
    bionics_json_path = ROOT / "bionics.json"
    var_name_json_path = ROOT /"var_names.json"
    wireless = WirelessProtocolLibrary(TcpCommunication(), bionics_json_path) # Time out meaning that the power knee is not connected
    
    # test 
    set_stance_flexion_level(wireless, 59)# initial stance flexion level
    set_toa_torque_level(wireless, 50) # ?? --> replaced by swing initiation
    set_swing_flexion_angle(wireless, 50) # target flexion angle
    
    print("DEFAULT initial stance flexion",get_stance_flexion_level(wireless))
    print("DEFAULT max flexion angle", get_swing_flexion_angle(wireless))
    print("DEFAULT swing initiation", get_toa_torque_level(wireless))

    
    monitor_and_feature_extraction(wireless, x_d = np.array([1., 1., 1.]))
