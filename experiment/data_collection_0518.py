import numpy as np
import os,sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.connection.udp import *
from src.connection.device import set_stance_flexion_level, set_swing_flexion_angle, set_toa_torque_level, get_stance_flexion_level, get_swing_flexion_angle, get_toa_torque_level
from src.ui import *


def main():
    cfg = get_json_data("experiments/DC_05_18.json")
    base_folder = cfg["base_folder_path"]
    os.makedirs(base_folder, exist_ok=True)

    while True:
        # SWF target angle (40-75), Swing init (0-100), Init STF Angle (0-100)
        # comb = [72,59,58]
        # comb = [72,59, 68] # 68 seems to be the best STF angle
        # comb = [72,59,78]  # 78 trip with 59
        # comb = [72, 69, 78] # 69 compatible wth 78
        # comb = [62, 69, 78] # ming is not fan of it, end it early, trip
        # comb = [67, 59, 58] # comfortable (is it 69, 78 or 59, 58?? check the history)
        # comb = [62, 59, 58] # compared to last time slow motion, more hip complensation, maybe step length increase 
        # comb = [62, 59, 68] # trips a lot, swing ext span decrease?
        # comb = [62, 69, 68] # 5 sec. trip reduced, faster swe
        # comb = [62, 79, 68] # comfortable
        # comb = [67, 69, 78] # good clearance, brake obvious
        comb = [67, 59, 68]
        # comb = [67, 59, 78] # good clearance, brake obvious




        print(f"experiment with control variables {[ctrl_var['name'] for ctrl_var in cfg['control_variables']]} = {comb}")
        

        log_file = os.path.join(base_folder, "_".join(str(int(comb[i])) for i in range(len(comb)))+".csv")
        print("save file to", log_file) 
        
        monitor_and_log_serial(log_file=log_file, log_time = cfg["log_time_per_session_sec"], log_premature=0, printlog = True)


if __name__ == "__main__":
    main()
    

