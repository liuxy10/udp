import numpy as np
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.connection.udp import *
from src.ui import *


def main():
    cfg = get_json_data("DC_04_26.json")
    base_folder = cfg["base_folder_path"]
    # tester = ProstheticKneeTester(import_protocol = True)
    os.makedirs(base_folder, exist_ok=True)

    while True:
        print(f"experiment with control variables {[ctrl_var['name'] for ctrl_var in cfg['control_variables']]}")
        
        comb = [69, 64, 80]
#         for i, ctrl_var in enumerate(cfg["control_variables"]):
#             print(f"getting {ctrl_var['name']}")
# #            tester.set_variable(ctrl_var["name_bionics"], int(comb[i]))
#             new_var = tester.get_variable(ctrl_var["name_bionics"])
#             comb.append(new_var)

        print(f"experiment with control variables {[ctrl_var['name'] for ctrl_var in cfg['control_variables']]} = {comb}")
        

        log_file = os.path.join(base_folder, "_".join(str(int(comb[i])) for i in range(len(comb)))+".csv")
        print("save file to", log_file) 
        
        monitor_and_log_serial(log_file=log_file, log_time = cfg["log_time_per_session_sec"], log_premature=0, printlog = True)


if __name__ == "__main__":
    main()
    

