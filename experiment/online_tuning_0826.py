import os, sys, time, pathlib
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from wireless_protocol_library import WirelessProtocolLibrary, TcpCommunication
from src.core import *
from src.connection.device import *
from src.connection.udp import *



if __name__ == "__main__":
    os.system('cls' if os.name == 'nt' else 'clear')
    
    BASE_DIR = pathlib.Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ENV_DIR = BASE_DIR / "env"
    bionics_json_path = ENV_DIR / "bionics.json"
    var_name_json_path = ENV_DIR / "var_names.json"
    DATA_DIR = pathlib.Path("~/Documents/Data/ossur").expanduser()

    wireless = WirelessProtocolLibrary(TcpCommunication(), bionics_json_path) # Time out meaning that the power knee is not connected
    exp_id = "OT_08_26/data" # Example path, adjust as needed
    save_folder = DATA_DIR / exp_id
    if not save_folder.exists():
        save_folder.mkdir(parents=True, exist_ok=True)

    # test
    set_stance_flexion_level(wireless, 50) # initial stance flexion level
    set_toa_torque_level(wireless, 50) # also known as swing initiation
    set_swing_flexion_angle(wireless, 49) # target flexion angle
    
    print("DEFAULT initial stance flexion",get_stance_flexion_level(wireless))
    print("DEFAULT max flexion angle", get_swing_flexion_angle(wireless))
    print("DEFAULT swing initiation", get_toa_torque_level(wireless))

    time_stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    # monitor_and_feature_extraction(wireless, x_d = np.array([1., 1., 1.]), vis = True, log_file = save_folder / f"log_{time_stamp}.csv")
    # constraint the activity to be ACTIVITY_FORWARD_PROG
    # set_activity(wireless, 1) # set activity to forward progression walking
    # Rerun the main logic to capture output
    # monitor(wireless, vis = True, log_file = save_folder / f"log_{time_stamp}.csv", change = [False, True, True])
    monitor(wireless, vis = True, log_file = "test.txt" , change = [True, False, True])