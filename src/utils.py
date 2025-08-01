import time
import numpy as np
import os, sys

import asciichartpy as acp
from src.connection.device import get_stance_flexion_level, get_swing_flexion_angle, get_toa_torque_level, set_stance_flexion_level, set_swing_flexion_angle, set_toa_torque_level, set_activity

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))


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

def set_user_parameters_batch(wireless, stance_flexion_level, swing_flexion_angle, toa_torque_level):
    """ Set the user parameters """
    while True: 
        try: 
            stance_flexion_level, swing_flexion_angle, toa_torque_level = int(min(max(stance_flexion_level,40), 75)), int(min(max(swing_flexion_angle, 40), 75)), int(min(max(toa_torque_level, 0), 100))
            set_stance_flexion_level(wireless, stance_flexion_level)  # initial stance flexion level
            set_swing_flexion_angle(wireless, swing_flexion_angle)  # target flexion angle
            set_toa_torque_level(wireless, toa_torque_level)  # ?? --> replaced by swing initiation

            if (get_stance_flexion_level(wireless) == stance_flexion_level and 
                get_swing_flexion_angle(wireless) == swing_flexion_angle and 
                get_toa_torque_level(wireless) == toa_torque_level):
                print("successfully update user parameters to ", np.array([stance_flexion_level, swing_flexion_angle, toa_torque_level]))
                os.system("afplay /System/Library/Sounds/Basso.aiff")  # play a sound to indicate update of user parameters
                break
            else: 
                print(f"fail assign param to be stance {stance_flexion_level} ({get_stance_flexion_level(wireless)}), swing {swing_flexion_angle} ({get_swing_flexion_angle(wireless)}), toa {toa_torque_level} ({get_toa_torque_level(wireless)})")
            
        except Exception as e:
            print(f"Error setting user parameters: {e}")
            time.sleep(0.1)

def set_user_parameters_action(wireless, action):
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
    c2 = min(max(c2_ + action[1], 40), 75)
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

