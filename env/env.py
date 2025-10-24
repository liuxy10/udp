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
        self.his_obs = []
        self.max_episode_steps = max_episode_steps
        self.reset()
        


    def initialize_puddle_boundaries(self, punish_bound):
        self.punish_bound = punish_bound 
        if punish_bound:
            low = self.observation_space.low
            high = self.observation_space.high
            n_bound_pts = 8
            # bottom edge
            bound = np.array([np.linspace(low[0], high[0], n_bound_pts), np.ones(n_bound_pts) * low[1]]).T
            self.puddle_center += bound.tolist()
            # top edge
            bound = np.array([np.linspace(low[0], high[0], n_bound_pts), np.ones(n_bound_pts) * high[1]]).T
            self.puddle_center += bound.tolist()
            # left edge
            bound = np.array([np.ones(n_bound_pts) * low[0], np.linspace(low[1], high[1], n_bound_pts)]).T
            self.puddle_center += bound.tolist()
            # right edge
            bound = np.array([np.ones(n_bound_pts) * high[0], np.linspace(low[1], high[1], n_bound_pts)]).T
            self.puddle_center += bound.tolist()

            self.puddle_width += [[0.08 * (high[0] - low[0]), 0.08 * (high[1] - low[1])] for _ in range(n_bound_pts*4)]
            


    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        self.pos = self.pos + action 
        pass
    


    def get_reward(self, state, action):
        reward = self.reward_map(state, action)
        # penalize if action would push pos out of bounds
        if self.punish_bound:
            next_pos = self.pos + action
            if not self.observation_space.contains(next_pos):
                reward -= 10.0
        return reward

    def _gaussian1d(self, p, mu, sig):
        return np.exp(-((p - mu)**2)/(2.*sig**2)) / (sig*np.sqrt(2.*np.pi))

    def reset(self):
        self.counter = 0
        if self.start is None:
            self.pos = self.param_space.sample()
        else:
            self.pos = copy.copy(self.start)
        # make sure to send out the reset state
        self.send_param()
        return self.pos
    

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
        # fill in the message by cfg name order, if not active, send default value
        message = {'params': {name: int(pos_full[i] * 100) for i, name in enumerate(self.cfg.param_names)}}
        print(f"Sending params via ZMQ: {message.values()}")
        self.publisher.send_json(message)

    def receive_state(self, vis=False, wait=5):
        """Receive state from ZMQ subscriber with a timeout."""
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
                        
                        return state
                    # If state is not valid, loop continues to get next message
            except zmq.Again:
                # This should not happen often with poll, but good to handle
                continue
        
        raise TimeoutError(f"Did not receive a valid state within {wait} seconds.")
        
