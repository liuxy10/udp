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

import numpy as np
import torch


		
def main(in_port = "4444", out_port = "3333", vis = True, 
         cfg = "/Users/xinyi/Documents/GitHub/udp/env/online_cfg.json"):
    
 
    
    cfg = Config(cfg)
    # env setup
    # reward function -- for real time cost evaluation
    reward_map = Feature2RewardQuadMap(Q = np.diag(np.ones(cfg.state_dim)), 
                                      R = np.diag(np.ones(cfg.action_dim) * 0.1), 
                                    #    target=state_mean.mean(axis=0))  # simple quadratic reward for testing
                                    target=cfg.normalized_target_values)  # simple quadratic reward for testing
    
    # initialize policy
     # build the trainer
    gamma, memory_size, memory_type, eval_type = cfg.lspi_cfg.gamma, cfg.lspi_cfg.memory_size, cfg.lspi_cfg.memory_type, cfg.lspi_cfg.eval_type

    # get a dummy env for agent initialization
    env = RealObsFeature(reward_map, 
                         goal = np.array(cfg.target_values), 
                         thrust=0.01, # IMPORTANT: step size of param update
                         start = [0.5, 0.5], # make sure aligning with core.py
                         cfg = cfg,
                         in_port = in_port, out_port = out_port,
                         max_episode_steps=10)
    
    # load pre-trained policy
    w = np.load(os.path.join(cfg.policy_path, "weights", "weights_rep_3.npz"), allow_pickle=True)["weights"]

    agent = lspi.agents.QuadraticAgent(env, w = w) # w = w for loading pre-trained policy
    baseline = lspi.baselines.LSPolicyIteration(env, agent, gamma, memory_size,
                                                        memory_type, eval_type)
    
    fig, axs, lines = None, None, []

    # start_time = time.time()
    # state_cur = env.receive_state( vis = True, wait = 100)
    # reward_cur = reward_map(state_cur, np.zeros(env.n_action))
    # done_cur = False
    # end_time = time.time()
    # print(f"Initial state received: {state_cur}, time taken: {end_time - start_time:.2f} seconds")

    print("---------------------------------------")
    print(f" Start RL Training")
    print("---------------------------------------")
    
    n_iter = 10

    for iter in range(n_iter):
        print(f"======== Iteration {iter+1} / {n_iter} ========")
        env.reset()
        print(len(env.memory), "!!!!!!!!!!")
        state_cur = env.receive_state( vis = True, wait = 100)
        reward_cur = reward_map(state_cur, np.zeros(env.n_action))
        done_cur = False
        while True:
            if env.counter % env.cfg.skip_num == 0: # when using the state to update
                print(f"Step: {env.counter}, current state: {state_cur}, current reward: {env.reward_map(state_cur, np.zeros(env.n_action))}")
                action = agent.predict(state_cur)
                # print(f"State: {state}, Action: {action}")
                state, reward, done, info = env.step(action) # update env with action and receive new state
                env.memory.append(Sample(s=state_cur, a=action, r=reward_cur, s_=state, done=done_cur))
                state_cur, reward_cur, done_cur = state, reward, done
                
                if done:
                    env.save_memory(env.cfg.policy_path)
                    baseline.agent.save(env.cfg.policy_path, name="weights_online")
                    print(("Episode finished after {} timesteps".format(env.counter)))
                    break
                
            else:
                # just receive new state without update
                state = env.receive_state( vis = True, wait = 10)
                env.counter += 1
        
        
        # policy update 
        w = baseline.agent.weights.copy()
        baseline.load_memory(env.memory)
        # baseline._batch_prep()


        # Policy update 
        for it in range(10): # max 10 policy updates
            baseline.train_step()
            print("w diff", np.linalg.norm(baseline.agent.weights - w))
            if np.linalg.norm(baseline.agent.weights - w) < 1e-4:
                break
            w = baseline.agent.weights.copy()
        
          

if __name__ == "__main__":
  
    main()