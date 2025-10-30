from collections import namedtuple
import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
import copy
import gym


import zmq 
import os,sys
sys.path.append(os.path.dirname(os.path.dirname(__file__))) # current repo root dir
from src.utils import vis_gait_cycle, feature_selection_from_mean_traj
import time
import matplotlib.pyplot as plt


Sample = namedtuple('Sample', ['s', 'a', 'r', 's_', 'done'])


class RealObsFeature(gym.Env): 
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, reward_map, cfg,
                start = None,
                goal=np.array([0,0,0]), 
                goal_threshold=0.1,
                noise=0.02, thrust=0.05,
                max_episode_steps=500,
                punish_bound = True,
                in_port = "4444", out_port = "3333"):
        
        
        self.reward_map = reward_map
        self.goal = goal
        assert len(goal) == self.reward_map.n_feature, "Goal dimension must match feature dimension"
        self.goal_threshold = goal_threshold
        self.noise = noise
        self.thrust = thrust
        self.n_action = self.reward_map.n_action
        self.n_state = self.reward_map.n_feature
        self.cfg = cfg
        self.action_space = spaces.Box(low = thrust * -np.ones(self.n_action), high = thrust * np.ones(self.n_action), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.array(self.cfg.state_limit)[:, 0], high=np.array(self.cfg.state_limit)[:, 1], dtype=np.float32) 
        self.param_space = spaces.Box(low=np.array(self.cfg.param_limit)[list(self.cfg.param_active.values()), 0], 
                                      high=np.array(self.cfg.param_limit)[list(self.cfg.param_active.values()), 1]) 

        self.start = start
        self.setup_zmq(in_port, out_port)
        self.punish_bound = punish_bound

        self.seed()
        self.counter = 0
        self.viewer = None
        self.memory = [] # history of observations per gait cycle
        self.max_episode_steps = max_episode_steps
        self.reset()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """Take an action in the environment each skip_num gait cycles.
        Args:
            action (np.ndarray): The action to take.
        Returns:
            tuple: A tuple containing the new state, reward, done flag, and info dictionary.
        """
        truncated = False
        # clip action to action space
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.pos = self.pos + action 
        # clip to param space
        self.send_param()
        reward = 0.0
        done = False
        if self.counter >= self.max_episode_steps:
            done = True
            truncated = True
        
        state = self.receive_state(vis=True, wait=10)
        if state is not None:
            self.counter += 1
            reward += self.reward_map(state, action) # TODO: distinguish current step/state vs next step/state?
            dist_to_goal = np.linalg.norm(state - self.goal)
            if dist_to_goal < self.goal_threshold:
                done = True
                reward += 50.0  # big reward for reaching goal

        return state, reward, done, {"truncated": truncated}



    def reset(self):
        self.counter = 0
        if self.start is None:
            self.pos = self.param_space.sample()
        else:
            self.pos = copy.copy(self.start)
        # make sure to send out the reset state
        self.send_param()
        self.state = self.receive_state(vis=False, wait=10)
        return self.pos, self.state
    
    ######### safety functions ##########
    
    def check_param_in_bound(self):
        for i, name in enumerate(self.cfg.param_names):
            if self.pos[i] < self.cfg.param_limit[i][0] or self.pos[i] > self.cfg.param_limit[i][1]:
                return False
        return True
    
    def _clip_param_in_bound(self, pos_full):
        for i, name in enumerate(self.cfg.param_names):
            pos_full[i] = np.clip(pos_full[i], self.cfg.param_limit[i][0], self.cfg.param_limit[i][1])
        return pos_full

    ########## communication functions ##########
    def setup_zmq(self, in_port, out_port):
        # ZMQ Context and Sockets
        context = zmq.Context() 
        # Socket to send messages out
        self.publisher = context.socket(zmq.PUB)
        self.publisher.bind(f"tcp://*:{out_port}")
        print(f"ZMQ Publisher bound to tcp://*:{out_port}")

        # Socket to receive messages
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.connect(f"tcp://localhost:{in_port}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "") # Subscribe to all topics
        print(f"ZMQ Subscriber connected to tcp://localhost:{in_port}")
    

    def send_param(self):
        
        pos_full = np.array([self.cfg.param_default[n] for n in self.cfg.param_names])
        active = np.array([self.cfg.param_active[n] for n in self.cfg.param_names])
        pos_full[active] = self.pos 
        pos_full = self._clip_param_in_bound(pos_full)
        # fill in the message by cfg name order, if not active, send default value
        message = {'params': {name: int(pos_full[i] * 100) for i, name in enumerate(self.cfg.param_names)}}
        print(f"Sending params via ZMQ: {list(message.values())}")
        self.publisher.send_json(message)

    def receive_state(self, vis=False, wait=5):
        """Receive state from ZMQ subscriber with a timeout.
        Args:
            vis (bool): Whether to visualize the gait cycle.
            wait (float): Maximum time to wait for a valid state in seconds.
        Returns:
            np.ndarray: The received state if valid, else None."""
        start_time = time.time()
        while time.time() - start_time < wait:
            try:
                # Poll for a message with a timeout based on remaining time
                remaining_time_ms = int((wait - (time.time() - start_time)) * 1000)
                if remaining_time_ms <= 0:
                    break  # Timeout exceeded

                if self.subscriber.poll(timeout=remaining_time_ms):
                    message = self.subscriber.recv_json(flags=zmq.NOBLOCK)
                    
                    if "trajectory" in message:
                        traj = message['trajectory']
                        
                        vis_gait_cycle(traj, self.cfg.targets) if vis else None
                       
                        state = feature_selection_from_mean_traj(traj, self.cfg.target_names)

                        state = np.array([(state[i] - self.cfg.state_limit[i][0]) / (self.cfg.state_limit[i][1] - self.cfg.state_limit[i][0]) 
                                    for i in range(self.cfg.state_dim)])
                        return state
                    # If state is not valid, loop continues to get next message
            except zmq.Again:
                # This should not happen often with poll, but good to handle
                continue
        
        raise TimeoutError(f"Did not receive a valid state within {wait} seconds.")
    


    ########## logging ##########
    def close(self):
        
        self.publisher.close()
        self.subscriber.close()
        
    def save_memory(self, folder_path):
        file_name = "online_memories_{}.csv".format(time.strftime("%Y%m%d-%H%M%S"))
        print(f"Saving environment memory to {os.path.join(folder_path, file_name)}")
        # Re-arrange the dataset into (state, action, reward, next_state) tuples for RL
        state_names = self.cfg.target_names if self.cfg.target_names else [f"s{i}" for i in range(self.n_state)]
        memories = []
        for sample in self.memory:
            s = sample.s
            a = sample.a
            r = sample.r
            s_ = sample.s_ if sample.s_ is not None else np.array([np.nan]*self.n_state)
            memories.append( np.concatenate( (s, s_, a, r) ) )
        # Save the generated transitions to a CSV file
        col_names = [f"s_{name}" for name in state_names] + [f"s_next_{name}" for name in state_names] + [f"a_c{i}" for i in range(self.n_action)] + ["r"]
        np.savetxt(os.path.join(folder_path, file_name), np.array(memories).reshape(len(memories), -1), header=",".join(col_names), delimiter=",")

        # --- Visualization ---
            
    
