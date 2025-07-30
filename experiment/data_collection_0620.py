import numpy as np
import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.connection.udp import *
from src.connection.device import set_stance_flexion_level, set_swing_flexion_angle, set_toa_torque_level, get_stance_flexion_level, get_swing_flexion_angle, get_toa_torque_level
from src.ui import *


def set_control_variables(wireless, params):
    """Set control variables for the wireless device."""
    c1, c2, c3 = int(params[0]), int(params[1]), int(params[2])
    c1 = min(max(c1, 40), 75) # stance flexion level
    c2 = min(max(c2, 0), 100)
    c3 = min(max(c3, 0), 100) # swing flexion angle
    while True: 
        try:   
            set_stance_flexion_level(wireless, c1)# initial stance flexion level
            set_swing_flexion_angle(wireless, c2) # target flexion angle
            set_toa_torque_level(wireless, c3) # replaced by swing initiation

            if get_stance_flexion_level(wireless) == c1 and get_swing_flexion_angle(wireless) == c2 and get_toa_torque_level(wireless) == c3:
                print("successfully update user parameters to ", np.array([c1, c2, c3]))
                break
        except Exception as e:
            print(f"Error setting user parameters: {e}")
            time.sleep(0.1)

def main():
    BASE_DIR = pathlib.Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ENV_DIR = BASE_DIR / "env"
    bionics_json_path = ENV_DIR / "bionics.json"
    var_name_json_path = ENV_DIR / "var_names.json"
    DATA_DIR = pathlib.Path("ossur").expanduser()

    wireless = WirelessProtocolLibrary(TcpCommunication(), bionics_json_path) # Time out meaning that the power knee is not connected
    
    cfg = get_json_data(DATA_DIR / "DC_06_20.json")
    base_folder = cfg["base_folder_path"]
    os.makedirs(base_folder, exist_ok=True)

    while True: # need smaller swing init 
        # SWF target angle (40-75), Swing init (0-100), Init STF Angle (0-100)
        # comb = [72,59,58]
        # comb = [72,59, 68] # 68 seems to be the best STF angle
        comb = [72,59,78]  # 78 trip with 59
        # comb = [72, 69, 78] # 69 compatible wth 78
        # comb = [62, 69, 78] # ming is not fan of it, end it early, trip
        comb = [40, 59, 58] # comfortable (is it 69, 78 or 59, 58?? check the history) 
        # comb = [62, 59, 58] # compared to last time slow motion, more hip complensation, maybe step length increase 
        # comb = [62, 59, 68] # trips a lot, swing ext span decrease?
        # comb = [62, 69, 68] # 5 sec. trip reduced, faster swe
        # comb = [62, 79, 68] # comfortable
        # comb = [67, 69, 78] # good clearance, brake obvious
        # comb = [67, 59, 68] # good clearance, brake obvious 
        # comb = [67, 59, 78] # good clearance, brake obvious


        ##### new parameters ######
        # comb = [40,59, 68] # strong swing extension; little scuff; 
        # comb = [40, 49, 68] # stong swing; no scuff
        # comb = [40, 49, 40] # strong braking
        # comb = [62, 49, 68]
        # # comb = [40, 49, 40] # vel reduce in the middle
        # comb = [40, 100, 30] 
        



        print(f"experiment with control variables {[ctrl_var['name'] for ctrl_var in cfg['control_variables']]} = {comb}")
        set_control_variables(wireless, comb)

        log_file = os.path.join(base_folder, "_".join(str(int(comb[i])) for i in range(len(comb)))+".csv")
        print("save file to", log_file) 
        
        monitor_and_log_serial(log_file=log_file, log_time = cfg["log_time_per_session_sec"], log_premature=0, printlog = True)


if __name__ == "__main__":
    main()
    

