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
import matplotlib
import io, asyncio, threading, websockets
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
        self.init_render()

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
        
        state = self.receive_state(vis=True, wait=10000)

        if state is not None:
            # self.counter += 1
            reward += self.reward_map(state, action) # TODO: distinguish current step/state vs next step/state?
            dist_to_goal = np.linalg.norm(state - self.goal)
            if dist_to_goal < self.goal_threshold:
                done = True
                reward += 50.0  # big reward for reaching goal
            
            # if out of bound, punish and reset
            if self.punish_bound:
                if not self.check_state_in_bound(state):
                    print("State out of bound! Resetting environment.")
                    reward -= 20.0
                    done = True
                

        return state, reward, done, {"truncated": truncated}



    def reset(self):
        if self.start is None:
            self.pos = self.param_space.sample()
        else:
            self.pos = copy.copy(self.start)
        # make sure to send out the reset state
        self.send_param()
        self.state = self.receive_state(vis=True, wait=10000)
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
    

    def check_state_in_bound(self, state):
        for i in range(self.n_state):
            if state[i] < -1 or state[i] > 1:
                return False
        return True
    

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
        
    def save_memory(self, folder_path, name = None):
        file_name = "online_memories_{}.csv".format(time.strftime("%Y%m%d-%H%M%S")) if name is None else name
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
        os.makedirs(folder_path, exist_ok=True)
        np.savetxt(os.path.join(folder_path, file_name), np.array(memories).reshape(len(memories), -1), header=",".join(col_names), delimiter=",")

    ####### rendering ########

    def init_render(self):
        matplotlib.use('Agg')  # off-screen backend

        # plotting skeleton: params (multiple lines), action (increments), states (trajectories), reward (trajectory)
        self.fig, axes = plt.subplots(4, 1, figsize=(8, 10), constrained_layout=True)
        ax_params, ax_action, ax_state, ax_reward = axes

        ax_params.set_title("Parameters (per active param)")
        ax_params.set_xlim(0, 200); ax_params.set_ylim(0, 1)
        ax_action.set_title("Action increments")
        ax_action.set_xlim(0, 200); ax_action.set_ylim(-1, 1)
        ax_state.set_title("State trajectories")
        ax_state.set_xlim(0, 200); ax_state.set_ylim(0, 1)
        ax_reward.set_title("Reward trajectory")
        ax_reward.set_xlim(0, 200); ax_reward.set_ylim(-50, 100)

        # create line objects; params: one line per full param set (will only plot active ones)
        n_params = len(self.cfg.param_names)
        self.param_lines = [ax_params.plot([], [], label=name)[0] for name in self.cfg.param_names]
        ax_params.legend(loc='upper right', fontsize='small')

        # action (plot each action dim as line)
        self.action_lines = [ax_action.plot([], [], label=f"a{i}")[0] for i in range(self.n_action)]
        ax_action.legend(loc='upper right', fontsize='small')

        # states
        self.state_lines = [ax_state.plot([], [], label=f"s{i}")[0] for i in range(self.n_state)]
        ax_state.legend(loc='upper right', fontsize='small')

        # reward single line
        self.reward_line, = ax_reward.plot([], [], label='reward')
        ax_reward.legend(loc='upper right', fontsize='small')

        # history buffers
        self._hist_max = 200
        self._time_buf = []
        self._param_buf = []  # list of length T of arrays length n_params
        self._action_buf = []  # list of length T of arrays length n_action
        self._state_buf = []  # list of length T of arrays length n_state
        self._reward_buf = []

        # helper to push new samples and update matplotlib line data
        def _push_sample(param, action, state, reward):
            t = (self._time_buf[-1] + 1) if self._time_buf else 0
            self._time_buf.append(t)
            self._param_buf.append(np.array(param))
            self._action_buf.append(np.array(action))
            self._state_buf.append(np.array(state) if state is not None else np.array([np.nan]*self.n_state))
            self._reward_buf.append(float(reward))

            # trim
            if len(self._time_buf) > self._hist_max:
                self._time_buf = self._time_buf[-self._hist_max:]
                self._param_buf = self._param_buf[-self._hist_max:]
                self._action_buf = self._action_buf[-self._hist_max:]
                self._state_buf = self._state_buf[-self._hist_max:]
                self._reward_buf = self._reward_buf[-self._hist_max:]

            xs = self._time_buf
            # update param lines
            params_arr = np.vstack(self._param_buf) if len(self._param_buf) else np.zeros((0, n_params))
            for i, line in enumerate(self.param_lines):
                ys = params_arr[:, i] if params_arr.size else []
                line.set_data(xs, ys)
                ax_params.relim(); ax_params.autoscale_view()

            actions_arr = np.vstack(self._action_buf) if len(self._action_buf) else np.zeros((0, self.n_action))
            for i, line in enumerate(self.action_lines):
                ys = actions_arr[:, i] if actions_arr.size else []
                line.set_data(xs, ys)
                ax_action.relim(); ax_action.autoscale_view()

            states_arr = np.vstack(self._state_buf) if len(self._state_buf) else np.zeros((0, self.n_state))
            for i, line in enumerate(self.state_lines):
                ys = states_arr[:, i] if states_arr.size else []
                line.set_data(xs, ys)
                ax_state.relim(); ax_state.autoscale_view()

            self.reward_line.set_data(xs, self._reward_buf)
            ax_reward.relim(); ax_reward.autoscale_view()

        # expose helper so step/render can push samples
        self._push_sample = _push_sample

        # WebSocket server to broadcast current PNG to connected clients
        self._ws_clients = set()
        self._ws_loop = asyncio.new_event_loop()

        async def _ws_handler(ws, path):
            self._ws_clients.add(ws)
            try:
                await ws.wait_closed()
            finally:
                self._ws_clients.discard(ws)

        def _start_ws_server():
            asyncio.set_event_loop(self._ws_loop)
            server = websockets.serve(_ws_handler, "0.0.0.0", 8765)
            self._ws_loop.run_until_complete(server)
            self._ws_loop.run_forever()

        t = threading.Thread(target=_start_ws_server, daemon=True)
        t.start()
        self._ws_thread = t

        def _broadcast_png():
            buf = io.BytesIO()
            self.fig.savefig(buf, format='png')
            data = buf.getvalue()
            if not self._ws_clients:
                return
            async def _send_all():
                coros = [client.send(data) for client in list(self._ws_clients)]
                await asyncio.gather(*coros, return_exceptions=True)
            asyncio.run_coroutine_threadsafe(_send_all(), self._ws_loop)

        self._broadcast_png = _broadcast_png

        print("Render initialized: Matplotlib skeleton created and WebSocket server started on ws://localhost:8765")

    def render(self, mode='human'):
        if mode == 'human':
            self._broadcast_png()
        else:
            raise NotImplementedError(f"Render mode '{mode}' not implemented.")

        # push current sample to history buffers
        self._push_sample(self.pos, np.zeros(self.n_action), self.state, 0.0)
        
