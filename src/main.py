import matplotlib.pyplot as plt
import numpy as np

import pandas as pd
import os, sys
from config import Config
from utils import vis_gait_cycle, feature_selection_from_mean_traj
sys.path.append(os.path.dirname(os.path.dirname(__file__))) # current repo root dir
from env.maps import ParamToFeatureMap, Feature2RewardQuadMap
from env.env import RealObsFeature 
import lspi
from collections import namedtuple
import time
"""
graphic interface to show msg from tcp (ZMQ)
"""
Sample = namedtuple('Sample', ['s', 'a', 'r', 's_', 'done'])


def main(in_port = "4444", out_port = "3333", vis = True, 
         cfg = "/Users/xinyi/Documents/GitHub/udp/env/online_cfg.json"):
    
 
    
    cfg = Config(cfg)
    # env setup
    # reward function -- for real time cost evaluation
    reward_map = Feature2RewardQuadMap(Q = np.diag(np.ones(cfg.state_dim)), 
                                      R = np.diag(np.ones(cfg.action_dim) * 0.1), 
                                    #    target=state_mean.mean(axis=0))  # simple quadratic reward for testing
                                    target=cfg.normalized_target_values)  # simple quadratic reward for testing
    
    fig = None
    axs = []
    lines = []


    # initialize policy
     # build the trainer
    gamma, memory_size, memory_type, eval_type = cfg.lspi_cfg.gamma, cfg.lspi_cfg.memory_size, cfg.lspi_cfg.memory_type, cfg.lspi_cfg.eval_type
    n_iter = 2 
    n_reps = 6
    use_prev_rep = False
    all_rewards = np.zeros((n_reps, n_iter + 1))
    # get a dummy env for agent initialization
    env = RealObsFeature(reward_map, goal = cfg.targets, cfg = cfg,
                         in_port = in_port, out_port = out_port)
    agent = lspi.agents.QuadraticAgent(env)
    baseline = lspi.baselines.LSPolicyIteration(env, agent, gamma, memory_size,
                                                        memory_type, eval_type)
    # load pre-trained policy
    # w = np.load("experiment/online_tuning_0826/pretrained_policy.npy")
    w = np.random.randn((cfg.state_dim + cfg.action_dim) * (cfg.state_dim + cfg.action_dim +1) //2) * 0.1
    agent.set_weights(w)

    start_time = time.time()
    state = env.receive_state( vis = False, wait = 100)
    end_time = time.time()
    print(f"Initial state received: {state}, time taken: {end_time - start_time:.2f} seconds")
    
    print("---------------------------------------")
    print(f" Start RL Training")
    print("---------------------------------------")
    

    while True:
        # Check for new parameters from ZMQ subscriber (non-blocking)
        try:
            state = env.receive_state( vis = False)
            if state is not None:
                pass

        except KeyboardInterrupt:
            print("Exiting...")
            break
                
                

            
            
        


if __name__ == "__main__":
    


    main()