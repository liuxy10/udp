import json
import logging
import os
import pathlib
import statistics
import time


from wireless_protocol_library import TcpCommunication, WirelessProtocolLibrary
# if running solely from the src directory
from connection.device import *
from plot_real_time import *

import tkinter as tk
from tkinter import ttk
from os import makedirs


class ProstheticKneeTester:
    """
    A class to test the functionality of a prosthetic knee using various tests.
    Attributes:
        ROOT (pathlib.Path): The root directory of the script.
        bionics_json_path (pathlib.Path): Path to the bionics JSON configuration file.
        var_name_json_path (pathlib.Path): Path to the variable names JSON configuration file.
        wireless (WirelessProtocolLibrary): An instance of the wireless protocol library for communication with the prosthetic knee.
    Methods:
        get_default_rt_var(col_name, state_appended=True):
            Retrieves the default real-time variables from the JSON configuration file.
        get_variable(variable_name):
            Retrieves the value of a specified variable from the prosthetic knee.
        set_variable(variable_name, value):
            Sets the value of a specified variable on the prosthetic knee.
        run_command_freq_test(frequency=50, duration=10):
            Tests the frequency of sending commands to the prosthetic knee.
        run_data_receiving_test(frequency=50, duration=60, var_name=None):
            Tests the frequency of receiving data from the prosthetic knee.
        run_latency_test(frequency=50, duration=5, var_name=None):
            Tests the latency of sending commands and receiving data from the prosthetic knee.
    """

    def __init__(self, import_protocol = True):

        BASE_DIR = pathlib.Path(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        ENV_DIR = BASE_DIR / "env"
        self.bionics_json_path = ENV_DIR / "bionics.json"
        self.var_name_json_path = ENV_DIR / "var_names.json"

        self._default_json_data = self.get_default_user_params_data()
        if import_protocol:
            try: 
                self.wireless = WirelessProtocolLibrary(TcpCommunication(), self.bionics_json_path) # Time out meaning that the power knee is not connected
            except Exception:
                print("NO CONNECTION TO THE POWERKNEE, if not in emulation, quit now!")
                
    def get_default_user_params_data(self):
        return get_json_data(self.var_name_json_path)
 
    def get_default_rt_var(self, col_name, state_appended = True):
        var_names = []
        
        try: 
            with open(self.var_name_json_path, 'r') as file:
                data = json.load(file)
            var_names += [item['name'] for item in data[col_name]]
        except KeyError:
            logging.error(f"Column name {col_name} not found in the JSON file")
        if state_appended:
            var_names += [
                "leg_ground_reaction_force".upper(),
                "knee_joint_angle".upper(),
                "base_gait_subphase".upper(), # we can use it to decide if the data 
                "base_gait_phase".upper(), # 0 for stance 1 for swing
                # "knee_stance_flexion_level_ground".upper()
                # "knee_flexion_target_angle".upper() # this variable time out during test
            ]
        return var_names

    def get_variable_range_default(self, col_name):
        var_range_dict = {}
        try: 
            with open(self.var_name_json_path, 'r') as file:
                data = json.load(file)
            for item in data[col_name]:
                var_range_dict[item['name']] = [item['min'], item['max'], item['default']]
        except KeyError:
            logging.error(f"Column name {col_name} not found in the JSON file")
            
        return var_range_dict
        
    
    def get_variable(self, variable_name):
        # Assuming self.wireless.bionics is the object containing the variables
        return self.wireless.get_variable(getattr(self.wireless.bionics, f"var_{variable_name.lower()}"))
    
    def set_variable(self, variable_name, value):
        # Assuming self.wireless.bionics is the object containing the variables
        return self.wireless.set_variable(getattr(self.wireless.bionics,  f"var_{variable_name.lower()}"), value)
    def run_command_freq_test(self, frequency=50, duration=10):  # Test the frequency of sending commands
        sleep_time = 1 / frequency
        start_time = time.time()
        end_time = start_time + duration
        command_count = 0

        logging.info("Starting Command Sending Test")
        while time.time() < end_time:
            # success = set_user_weight(self.wireless, 10)
            success = set_user_thigh_length(self.wireless, 10)
            if success:
                command_count += 1
            time.sleep(sleep_time)

        actual_frequency = command_count / duration
        logging.info(f"Command Test Results: Sent {command_count} commands in {duration} seconds")
        logging.info(f"Achieved frequency: {actual_frequency:.2f} Hz")

    def run_data_receiving_test(self, frequency=50, duration=60, var_name = None):
        sleep_time = 1 / frequency
        start_time = time.time()
        end_time = start_time + duration
        data_count = 0
        data = []
        
        if var_name is None:
            var_name = self.get_default_rt_var('default_column')[0]

        logging.info("Starting Data Receiving Test")
        while time.time() < end_time:
            d = self.get_variable(var_name)
            data.append(d)
            data_count += 1

        actual_frequency = data_count / duration
        logging.info(f"Data Test Results: Received {data_count} data points in {duration} seconds")
        logging.info(f"Achieved frequency: {actual_frequency:.2f} Hz")
        logging.info(f"Average knee angle: {statistics.mean(data):.2f} degrees")

    def run_latency_test(self, frequency=50, duration=5, var_name = None):
        sleep_time = 1 / frequency
        start_time = time.time()
        end_time = start_time + duration
        latencies = []
        if var_name == None:
            var_name = self.get_default_rt_var('default_column')[0]

        logging.info("Starting Latency Test")
        while time.time() < end_time:
            cmd_start = time.time()
            # self.wireless.send_command("set_position")
            data = self.get_variable(var_name)
            cmd_end = time.time()

            latency = (cmd_end - cmd_start) * 1000  # Convert to milliseconds
            latencies.append(latency)
            time.sleep(sleep_time)

        avg_latency = statistics.mean(latencies)
        max_latency = max(latencies)
        min_latency = min(latencies)
        logging.info(f"Latency Test Results: Avg: {avg_latency:.2f} ms, Min: {min_latency:.2f} ms, Max: {max_latency:.2f} ms")


  

# matplotlib.use('Agg')  # Use a non-GUI backend
def get_json_data(path):
    with open(path) as f:
        data = json.load(f)
    return data

def save_json_data(path, data):
    with open(path, 'w') as f:
        json.dump(data, f, indent=4)

