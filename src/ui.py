import json
import logging
import os
import pathlib
import time
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from wireless_protocol_library import TcpCommunication, WirelessProtocolLibrary

from connection.device import *
from plot_real_time import *
from connection.tcp import ProstheticKneeTester

import tkinter as tk
from tkinter import ttk
from os import makedirs


# matplotlib.use('Agg')  # Use a non-GUI backend
def get_json_data(path):
    with open(path) as f:
        data = json.load(f)
    return data

def save_json_data(path, data):
    with open(path, 'w') as f:
        json.dump(data, f, indent=4)


class ProstheticKneeUI:
    """
    A class to create a UI for setting values for the prosthetic knee.
    Attributes:
        root (tk.Tk): The root window of the UI.
        tester (ProstheticKneeTester): An instance of the ProstheticKneeTester class.
    Methods:
        create_ui():
            Creates the UI with toggle bars for setting values.
        set_value(variable_name, value):
            Sets the value of a specified variable on the prosthetic knee.
    """

    def __init__(self, tester, exp_name = 'temp', col_name = "PK_RW_walk"):
        self.root = tk.Tk()
        self.root.title("Prosthetic Knee Control Interface")
        self.tester = tester
        self.exp_name = exp_name
        self.col_name = col_name
        self.exp_log_path = pathlib.Path(os.path.dirname(os.path.abspath(__file__))) / "logs" / "11_11_2024"
        self.exp_json_path = self.exp_log_path/pathlib.Path(self.exp_name + ".json")
        
        self.var_names = self.tester.get_default_rt_var(col_name, state_appended = False)
        self.var_dict = {}
        self.var_range = self.tester.get_variable_range_default(col_name)
        self._default_json_data = self.tester.get_default_user_params_data()
        print(f"[UI] Log data into {self.exp_log_path}")
        if not os.path.exists(self.exp_log_path):
            makedirs(self.exp_log_path, exist_ok=True)
        if not os.path.exists(self.exp_json_path ):
            save_json_data(self.exp_log_path/pathlib.Path(self.exp_name + ".json"), self._default_json_data)
        self._changed_json_data = get_json_data(self.exp_json_path)
        
        

        
    def create_ui(self):
        # Create a frame for the toggle bars
        frame = ttk.Frame(self.root, padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.start_time = time.time()
        for i, var_name in enumerate(self.var_names):
            ttk.Label(frame, text=f"{var_name}").grid(row=i, column=0, sticky=tk.W)
            self.var_dict[var_name] = tk.DoubleVar()
            scale = tk.Scale(frame, variable=self.var_dict[var_name], from_=self.var_range[var_name][0], to= self.var_range[var_name][1] , orient=tk.HORIZONTAL, resolution=1)
            scale.set(self._changed_json_data[self.col_name][i]['default'])  # Set the initial value to the default
            scale.grid(row=i, column=1, sticky=(tk.W, tk.E))
            scale.config(length=400)  # Make the bar longer
            ttk.Label(frame, textvariable=self.var_dict[var_name], text=f"{self.var_dict[var_name].get():.3f}").grid(row=i, column=2, sticky=tk.W)
            ttk.Label(frame, text=f"(Min: {self.var_range[var_name][0]}, Max: {self.var_range[var_name][1]})").grid(row=i, column=3, sticky=tk.W)

        # Create a button to set the values
        set_button = ttk.Button(frame, text="Set Values", command=self.set_values)
        set_button.grid(row=len(self.var_names), column=0, columnspan=4, pady=10)

        self.root.mainloop()

    def set_values(self):
        print("*"*50)
        for var_name in self.var_names:
            logging.info(f"At t = {time.time() - self.start_time}, Set {var_name} to {self.var_dict[var_name].get()}")
            self.tester.set_variable(var_name, int(self.var_dict[var_name].get()))
        # update the json data with the new values
        for var_config in self._changed_json_data[self.col_name]:
            # print(var_config,  self.var_dict[var_name].get())
                var_config['default'] =  self.var_dict[var_config['name']].get()
        # save current var into a json file called temp.json
        save_json_data(self.exp_log_path/pathlib.Path(self.exp_name + ".json"), self._changed_json_data) 


        
        
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    # app = QApplication(sys.argv)
    tester = ProstheticKneeTester()
    # tester.run_command_freq_test()
    # tester.run_data_receiving_test()
    # tester.run_latency_test()

    ui = ProstheticKneeUI(tester, exp_name = 'test', col_name = "PK_RW_walk")  # Assuming tester and col_name are defined
    ui.create_ui()

      