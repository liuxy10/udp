import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__))) # 
from udp_core import *
# import lspi 
import time
import os
from ui import *
import json
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
    

