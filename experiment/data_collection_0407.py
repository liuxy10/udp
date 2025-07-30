import numpy as np
import os, sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from src.connection.udp import *
from src.ui import *


def main():
    cfg = get_json_data("DC_04_07.json")
    base_folder = cfg["base_folder_path"]
    tester = ProstheticKneeTester(import_protocol = True)
    os.makedirs(base_folder, exist_ok=True)
    
    if False:
        control_values = []
        for ctrl_var in cfg["control_variables"]:
            control_values.append(np.linspace( ctrl_var["min"],ctrl_var["max"], num=ctrl_var['number of steps']))
        
        # Generate all possible combinations
        control_combinations = np.array(np.meshgrid(*control_values)).T.reshape(-1, len(control_values))
        # shuffle the combinations
        control_combinations = control_combinations[np.random.permutation(control_combinations.shape[0])]
        print(control_combinations)
    
    control_combinations = np.array([
                    [ 25.,    75,     100.],
                    [ 25.,    40.,    75. ],
                    [ 75.,    66.25,  75. ],
                    [100.,    48.75,  75. ],
                    [100.,    40.,     0. ],
                    [ 75.,    40.,    50. ],
                    [100.,    66.25,  75. ],
                    [100.,    57.5,   50. ],
                    [  0.,    57.5,    0. ],
                    [ 50.,    40.,     0. ],# from this one, not collecting premature 30 period
                    [ 50.,    40.,   100. ],
                    [ 75.,    75.,   100. ],
                    [  0.,    48.75,  75. ],
                    [ 50.,    75.,     0. ],
                    [ 25.,    48.75,  25. ],
                    [ 75.,    66.25,  50. ],
                    [100.,    48.75,   0. ],
                    [ 25.,    66.25,  75. ],
                    [ 75.,    66.25, 100. ],
                    [  0.,    66.25,  50. ],
                    [  0.,    40.,   100. ],
                    [100.,    75.,     0. ],
                    [100.,    40.,    50. ],
                    [ 50.,    75.,    50. ],
                    [ 50.,    48.75,   0. ],
                    [ 50.,    57.5,    0. ],
                    [  0.,    57.5,  100. ],
                    [ 75.,    66.25,   0. ],
                    [ 50.,    57.5,  100. ],
                    [  0.,    48.75,  50. ],
                    [100.,    66.25, 100. ],
                    [ 75.,    75.,    25. ],
                    [  0.,    57.5,   25. ],
                    [ 75.,    57.5,    0. ],
                    [ 25.,    75.,     0. ],
                    [ 50.,    66.25,  75. ],
                    [ 25.,    40.,     0. ],
                    [ 75.,    57.5,  100. ],
                    [ 75.,    75.,     0. ],
                    [  0.,    48.75, 100. ],
                    [ 50.,    66.25,  25. ],
                    [  0.,    66.25,  75. ],
                    [ 50.,    66.25,  50. ],
                    [ 75.,    40.,   100. ],
                    [100.,    40.,   100. ],
                    [  0.,    40.,    50. ],
                    [  0.,    48.75,   0. ],
                    [ 50.,    48.75, 100. ],
                    [ 50.,    48.75,  75. ],
                    [100.,    40.,    75. ],
                    [ 25.,    48.75,  50. ],
                    [ 75.,    75.,    50. ],
                    [ 25.,    40.,    50. ],
                    [  0.,    75.,    25. ],
                    [  0.,    75.,    75. ],
                    [100.,    75.,    75. ],
                    [  0.,    48.75,  25. ],
                    [ 50.,    57.5,   75. ],
                    [  0.,    40.,    25. ],
                    [ 50.,    57.5,   25. ],
                    [ 25.,    48.75,   0. ],
                    [100.,    57.5,   25. ],
                    [ 50.,    75.,    75. ],
                    [100.,    66.25,  25. ],
                    [ 25.,    57.5,    0. ],
                    [100.,    48.75,  25. ],
                    [ 25.,    66.25,  50. ],
                    [ 75.,    48.75,   0. ],
                    [  0.,    66.25,   0. ],
                    [100.,    57.5,  100. ],
                    [  0.,    75.,   100. ],
                    [ 25.,    66.25,   0. ],
                    [ 75.,    40.,     0. ],
                    [100.,    75.,    50. ],
                    [100.,    48.75, 100. ],
                    [ 25.,    75.,    50. ],
                    [ 25.,    66.25, 100. ],
                    [100.,    66.25,   0. ],
                    [100.,    66.25,  50. ],
                    [100.,    75.,    25. ],
                    [ 25.,    57.5,   25. ],
                    [ 75.,    48.75,  75. ],
                    [  0.,    66.25, 100. ],
                    [ 25.,    48.75,  75. ],
                    [ 75.,    75.,    75. ],
                    [ 75.,    48.75,  50. ],
                    [ 50.,    75.,    25. ],
                    [ 75.,    57.5,   50. ],
                    [ 50.,    57.5,   50. ],
                    [ 75.,    57.5,   25. ],
                    [100.,    40.,    25. ],
                    [ 50.,    48.75,  50. ],
                    [100.,    57.5,   75. ],
                    [ 50.,    40.,    75. ],
                    [ 50.,    40.,    25. ],
                    [  0.,    57.5,   50. ],
                    [  0.,    75.,     0. ],
                    [ 75.,    40.,    75. ],
                    [100.,    48.75,  50. ],
                    [100.,    75.,   100. ],
                    [  0.,    66.25,  25. ],
                    [ 25.,    57.5,   50. ],
                    [ 25.,    66.25,  25. ],
                    [ 25.,    75.,    75. ],
                    [ 50.,    40.,    50. ],
                    [ 25.,    40.,    25. ],
                    [ 25.,    48.75, 100. ],
                    [  0.,    75.,    50. ],
                    [ 75.,    40.,    25. ],
                    [  0.,    57.5,   75. ],
                    [ 75.,    66.25,  25. ],
                    [ 25.,    40.,   100. ],
                    [ 75.,    57.5,   75. ],
                    [ 25.,    57.5,  100. ],
                    [ 25.,    57.5,   75. ],
                    [ 75.,    48.75, 100. ],
                    [100.,    57.5,    0. ],
                    [ 50.,    75.,   100. ],
                    [ 50.,    66.25, 100. ]
                ])
     # control_combinations = np.array([                   


    # exit(0)
    for comb in control_combinations:
        
        #set the control variables
        print(f"experiment with control variables {[ctrl_var['name'] for ctrl_var in cfg['control_variables']]} =  {comb}")
        
        for i, ctrl_var in enumerate(cfg["control_variables"]):
            tester.set_variable(ctrl_var["name_bionics"], int(comb[i]))
            new_var = tester.get_variable(ctrl_var["name_bionics"])
            # assert new_var == int(comb[i]), f"did not set {ctrl_var['name']} to the value desired: {comb[i]}, now {new_var}"
            if new_var != int(comb[i]):
                while new_var != int(comb[i]):
                    time.sleep(2)
                    new_var = tester.get_variable(ctrl_var["name_bionics"])
                    print(f"did not set {ctrl_var['name']} to the value desired: {comb[i]}, now {new_var}, pls set it via app on mobile")

        log_file = os.path.join(base_folder, "_".join([str(int(comb[i])) for i in range(len(comb))])+".csv")
        
        
        monitor_and_log_serial(log_file=log_file, log_time = cfg["log_time_per_session_sec"], log_premature=cfg["log_premature_time_per_session_sec"])

    



    

if __name__ == "__main__":
    main()
    
