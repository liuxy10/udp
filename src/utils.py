import time
import numpy as np
import os, sys
import pandas as pd
import asciichartpy as acp
# add path of ossur_knee in github repo
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), '..'))
from ossur_knee.src.utils.feature_selection import extract_features
# add path of src
sys.path.append(os.path.join(os.path.dirname(__file__)))
from connection.device import get_stance_flexion_level, get_swing_flexion_angle, get_toa_torque_level, set_stance_flexion_level, set_swing_flexion_angle, set_toa_torque_level, set_activity
import numpy as np

import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.interpolate import CubicHermiteSpline
# Global variables to hold the figure and axes for interactive plotting
fig, axs = None, None
lines = []


def popFirstNSamples(dict, n):
    """ Pop the first n samples from the history """
    n = min(n, len(dict[list(dict.keys())[0]]))
    return {k: v[n:] for k, v in dict.items()}

# def feature_selection_from_mean_traj(traj_buffer, target_names=None, vis = False): # for multiple gait cycles
#     """ Extract features from the trajectory buffer """
#     mean = {k: [] for k in traj_buffer.keys()}  # initialize mean dictionary
#     i = 0
#     for k in traj_buffer.keys():
#         for traj in traj_buffer[k]:
#             if len(traj) > 0: # Interpolate to handle missing or unevenly sampled data
#                 traj = np.array(traj)
#                 x = np.linspace(0, 1, len(traj))
#                 x_uniform = np.linspace(0, 1, 100)
#                 traj_interp = np.interp(x_uniform, x, traj)
#                 mean[k].append(traj_interp)
#         mean[k] = np.mean(np.array(mean[k]), axis=0)  # take the mean of the trajectories
#         assert len(mean[k]) == 100, f"Mean trajectory for {k} is not of length 100, instead, got {len(mean[k])}"
#     return feature_selection_per_gait(mean, vis)  # extract features from the mean trajectory



def feature_selection_per_gait(dat, vis): # for one gait cycle
    dat = smooth_data(dat, window_size=5, names = ["LOADCELL", "ACTUATOR_POSITION"]) # smooth the data
    # if vis: 
    #     vis_gait_cycle_terminal(dat)
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


def smooth_data(data, window_size=5, names = ["LOADCELL", "ACTUATOR_POSITION"]):
    """ Smooth the data using a simple moving average"""
    for name in names:
        if name in data.keys():
            data[name] = np.convolve(data[name], np.ones(window_size)/window_size, mode='valid')
    return data



def feature_selection_from_mean_traj(traj_buffer, target_names=[]): # for multiple gait cycles
    traj_buffer_mean = {k: process_signal(traj_buffer[k])[0] for k in traj_buffer.keys()}  # take the mean of the trajectories
    features = extract_features([pd.DataFrame(traj_buffer_mean)], target_names=target_names)[0]
    # dict name and feature value

    return np.array(features) # use same function with offline learning




def process_signal(data):
    interpolated = np.array([np.interp(np.linspace(0, len(s) - 1, 100), np.arange(len(s)), s) for s in data])
    mean = interpolated.mean(axis=0)
    std = interpolated.std(axis=0)
    return mean, std


def vis_gait_cycle(traj_buffer, targets = {}):
    """
    Visualize gait cycle data using matplotlib for interactive plotting.
    This function initializes a plot on the first call and updates it
    on subsequent calls with new gait cycle data.
    """
    global fig, axs, lines

    # Extract data from the buffer, interpolate to 100 points, and calculate mean and std
    target_vis = {
            "max_knee_position_st": 20 + 20,
            "max_knee_position_phase_st": 0.1,
            "min_knee_position_st": 5 + 20,
            "min_knee_position_phase_st": 0.4,
            "st_sw_phase": 0.6,
            "max_knee_position_sw": 70 + 20,
            "max_knee_position_phase_sw": 0.7 - 0.6,
            "min_knee_position_sw": 0 + 20,
            "min_knee_position_phase_sw": 0.95 - 0.6
         }
    
    # update the value of target_vis using targets
    for k in target_vis.keys():
        if k in targets:
            target_vis[k] = targets[k]
    

    loadcell_mean, loadcell_std = process_signal(traj_buffer ["GAIT_SUBPHASE"]) #["LOADCELL"])
    knee_angle_mean, knee_angle_std = process_signal(traj_buffer["ACTUATOR_POSITION"])
    torque_mean, torque_std = process_signal(traj_buffer["TORQUE_ESTIMATE"])
    subphase_mean, _ = process_signal(traj_buffer["GAIT_SUBPHASE"])
    all_data_mean = [loadcell_mean, torque_mean, knee_angle_mean]
    all_data_std = [loadcell_std, torque_std, knee_angle_std]
    x_axis = np.linspace(0, 1, len(knee_angle_mean))
    st_sw_transition = np.argmax(subphase_mean > 2)/100  # Example threshold for stance to swing transition
    # classic min, max points 
    y_names = ["max_knee_position_st", "min_knee_position_st", "max_knee_position_sw", "min_knee_position_sw"]
    x_names = ["max_knee_position_phase_st", "min_knee_position_phase_st", "max_knee_position_phase_sw", "min_knee_position_phase_sw"]

    titles = ["subphase", "Torque Estimate", "Knee Angle (Actuator Position)"]
    ylabels = ["", "Torque (Nm)", "Angle (degrees)"]
    colors = ['green', 'blue', 'red']
    # If the plot has not been created yet, initialize it
    if fig is None:
        plt.ion()  # Turn on interactive mode
        fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        fig.suptitle('Gait Cycle Analysis', fontsize=16)
        fig.show()

        fig, axs = plot_lines(fig, axs, all_data_mean, all_data_std, x_axis, st_sw_transition, titles, ylabels, colors)
        
        # feature points
        ys = feature_selection_from_mean_traj(traj_buffer, target_names=y_names)
        xs = feature_selection_from_mean_traj(traj_buffer, target_names=x_names)
        sw_indices = [i for i, name in enumerate(x_names) if 'sw' in name]
        xs[sw_indices] += st_sw_transition

        axs = vis_feature(axs, ys, xs, y_names=y_names)
        
    
        axs[2].set_xlabel("Gait phase (0-1)")
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    # If the plot already exists, update the data for each line
    for i, line in enumerate(lines):
        line.set_xdata(x_axis)
        line.set_ydata(all_data_mean[i])
        # update the shaded region for +/- 3 std
        for collection in axs[i].collections:
            collection.remove()
        axs[i].fill_between(x_axis, all_data_mean[i] - 3 * all_data_std[i], all_data_mean[i] + 3 * all_data_std[i], color=line.get_color(), alpha=0.2)
        # update the vertical line for stance to swing transition
        for ax_line in axs[i].lines:
            if ax_line.get_linestyle() == '--':
                ax_line.set_xdata([st_sw_transition, st_sw_transition])
        # feature and targets
        if i == 2:  # Only update feature points on the knee angle subplot
            # Remove existing feature points and texts
            for txt in axs[i].texts:
                txt.remove()
            # Remove old feature points (dots)
            # The main data line and the vertical line are kept, others are removed.
            lines_to_remove = [l for l in axs[i].lines if l not in lines and l.get_linestyle() != '--']
            for l in lines_to_remove:
                l.remove()
            # Plot new feature points
            ys = feature_selection_from_mean_traj(traj_buffer, target_names=y_names)
            xs = feature_selection_from_mean_traj(traj_buffer, target_names=x_names)
            sw_indices = [i for i, name in enumerate(x_names) if 'sw' in name]
            xs[sw_indices] += st_sw_transition
            axs = vis_feature(axs, ys, xs, y_names)
            axs = vis_target_line(axs, target_vis)

        # Add legend    
            axs[i].legend(loc='upper right', fontsize='x-small')
        # Autoscale the axes to fit the new data
        axs[i].relim()
        axs[i].autoscale_view()


    # Redraw the canvas and process GUI events
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.005) 

def vis_feature(axs, ys, xs, y_names):
    for j, (x, y) in enumerate(zip(xs, ys)):
        name = y_names[j].split('_')[0]+"_"+ y_names[j].split('_')[-1]
        color = 'red' if 'st' in name else 'green'
        marker = 'o' if 'max' in name else 'x'
        axs[2].plot(x, y, color=color, marker=marker)
        axs[2].text(x, y, name, color=color)
    return axs

def plot_lines(fig, axs, all_data_mean, all_data_std, x_axis, st_sw_transition, titles, ylabels, colors):
    for i, ax in enumerate(axs):
            # Create line objects with initial data. We will update their data later.
        line, = ax.plot(x_axis, all_data_mean[i], color=colors[i], label=titles[i])
        lines.append(line)
            # Add shaded region for +/- 3 std
        ax.fill_between(x_axis, all_data_mean[i] - 3 * all_data_std[i], all_data_mean[i] + 3 * all_data_std[i], color=colors[i], alpha=0.2)
            # vertical line for stance to swing transition
        ax.axvline(x=st_sw_transition, color='gray', linestyle='--', label='Stance-Swing Transition')
                  
        ax.set_title(titles[i])
        ax.set_ylabel(ylabels[i])
        ax.grid(True)
        ax.legend(loc='upper left', fontsize='x-small') # A short pause to allow the plot to update
    return fig, axs

def vis_target_line(axs, targets):
    # clean up previous target lines if there are any
    for ax in axs:
        lines_to_remove = [l for l in ax.lines if l.get_linestyle() == 'k-']
        for l in lines_to_remove:
            l.remove()
    # target line (spline)
    control_points = []
    for i, (name, value) in enumerate(targets.items()):
        # pair value and phase with the same name in target as control points of spline
        if 'phase' not in name or 'transition' in name:
            continue
        value_name = name.replace('_phase', '')
        if value_name in targets:
            phase_value = targets[name] + targets["st_sw_phase"] if 'sw' in name else targets[name]
            axs[2].plot(phase_value, targets[value_name], 'kx')  # plot control points
            axs[2].text(phase_value, targets[value_name], f'{value_name}', color='black')
            control_points.append((phase_value, targets[value_name]))
    
    control_points = sorted(control_points)  # sort by phase
    # plot spline
    x_sp = np.linspace(0, 1, 100)
    if len(control_points) >= 2:
        # Make the spline periodic by ensuring the start and end y-values are the same.
        if control_points:
            control_points.append((1.0, control_points[0][1])) # Add end point to match start point y-value
            control_points.append((0.0, control_points[0][1])) # Add start point to match end point y-value
            control_points = sorted(list(set(control_points))) # sort and remove duplicates

        cp_x, cp_y = zip(*control_points)
        # Create a spline where the derivative at each control point is zero, making them local extrema.
        # This is achieved using a Cubic Hermite Spline where the derivative at each knot is set to 0.
        dy = np.zeros_like(cp_y)
        cs = CubicHermiteSpline(cp_x, cp_y, dy, extrapolate=False)
        axs[2].plot(x_sp, cs(x_sp), 'k-', alpha=0.2, label='Target Spline')
        axs[2].axvline(x=targets["st_sw_phase"], color='gray', linestyle='-', alpha=0.2, label='target Transition')
        axs[2].legend(loc='upper right', fontsize='x-small')
    return axs

if __name__ == "__main__":
    pass
