import matplotlib.pyplot as plt
import numpy as np
import serial
from udp_core import SERIAL_PORT, BAUD_RATE, START_BYTE, PACKET_SIZE, parse_packet, get_shank_angle_from_gravity_vec_degree
from pynput import keyboard
import time


pause = False
delete_prev = False
reset = False


def on_press(key):
    """Handle key press events."""
    global pause, reset, delete_prev
    if key == keyboard.Key.space:
        pause = not pause
        reset = False
        print(f'Key {key} pressed, pause state: {pause}')
    if key == keyboard.Key.delete:
        delete_prev = not delete_prev
        print(f'Key {key} pressed, delete previous positions')


class RealTimeLegPlotter:
    def __init__(self, ax, thigh_length=0.4, shank_length=0.4, foot_length=0.15):
        self.ax = ax
        self.thigh_length = thigh_length


class RealTimeLegPlotter:
    def __init__(self, ax, thigh_length=0.4, shank_length=0.4, foot_length=0.15):
        self.ax = ax
        self.thigh_length = thigh_length
        self.shank_length = shank_length
        self.foot_length = foot_length

        self.line_thigh, = ax.plot([], [], 'r-', linewidth=2, label='Thigh')
        self.line_shank, = ax.plot([], [], 'b-', linewidth=2, label='Shank')
        self.foot_patch = plt.Polygon([[0,0],[0,0],[0,0]], closed=True, color='g', alpha=0.7, label='Foot')
        self.ax.add_patch(self.foot_patch)
        self.joints = ax.scatter([], [], c='k', zorder=5)
        self.texts = []
        self.ax.axis('equal')
        self.ax.set_xlim(-0.8, 0.8)
        self.ax.set_ylim(-0.9, 0.2)
        self.ax.set_xlabel('X position (m)')
        self.ax.set_ylabel('Y position (m)')
        self.ax.set_title('Real-Time Leg Sagittal Plane')
        self.ax.grid(True)
        self.ax.legend()

    def reinitialize(self):
        self.ax.clear()
        self.line_thigh, = self.ax.plot([], [], 'r-', linewidth=2, label='Thigh')
        self.line_shank, = self.ax.plot([], [], 'b-', linewidth=2, label='Shank')
        self.foot_patch = plt.Polygon([[0,0],[0,0],[0,0]], closed=True, color='g', alpha=0.7, label='Foot')
        self.ax.add_patch(self.foot_patch)
        self.joints = self.ax.scatter([], [], c='k', zorder=5)
        self.texts = []
        self.ax.axis('equal')
        self.ax.set_xlim(-0.8, 0.8)
        self.ax.set_ylim(-0.9, 0.2)
        self.ax.set_xlabel('X position (m)')
        self.ax.set_ylabel('Y position (m)')
        self.ax.set_title('Real-Time Leg Sagittal Plane')
        self.ax.grid(True)
        self.ax.legend()

    def update(self, knee_angle, shank_angle):
        # Clear previous annotations
        for txt in self.texts:
            txt.remove()
        self.texts.clear()

        # Calculate limb positions
        thigh_rad = np.radians(shank_angle + knee_angle)
        hip = (0, 0)
        knee = (
            self.thigh_length * np.sin(thigh_rad),
            -self.thigh_length * np.cos(thigh_rad)
        )
        ankle = (
            knee[0] + self.shank_length * np.sin(np.radians(shank_angle)),
            knee[1] - self.shank_length * np.cos(np.radians(shank_angle))
        )

        # Calculate foot points
        shank_rad = np.radians(shank_angle)
        heel = (
            ankle[0] - self.foot_length/2 * np.cos(shank_rad),
            ankle[1] - self.foot_length/2 * np.sin(shank_rad)
        )
        toe = (
            ankle[0] + self.foot_length * np.cos(shank_rad),
            ankle[1] + self.foot_length * np.sin(shank_rad)
        )
        
        # Update plot elements
        self.line_thigh.set_data([hip[0], knee[0]], [hip[1], knee[1]])
        self.line_shank.set_data([knee[0], ankle[0]], [knee[1], ankle[1]])
        self.joints.set_offsets([hip, knee, ankle])
        self.foot_patch.set_xy(np.array([heel, ankle, toe]))

        # Add dynamic annotations
        self.texts.append(self.ax.text(*hip, 'Hip', ha='right', fontsize=8))
        self.texts.append(self.ax.text(*knee, 'Knee', ha='right', fontsize=8))
        self.texts.append(self.ax.text(*ankle, 'Ankle', ha='right', fontsize=8))
        self.texts.append(self.ax.text(*heel, 'Heel', ha='left', fontsize=8))
        self.texts.append(self.ax.text(*toe, 'Toe', ha='left', fontsize=8))
        
    def add_prev_positions(self, knee_angle, shank_angle, color = 'grey', alpha = 0.5):
        # Calculate limb positions
        thigh_rad = np.radians(shank_angle + knee_angle)
        hip = (0, 0)
        knee = (
            self.thigh_length * np.sin(thigh_rad),
            -self.thigh_length * np.cos(thigh_rad)
        )
        ankle = (
            knee[0] + self.shank_length * np.sin(np.radians(shank_angle)),
            knee[1] - self.shank_length * np.cos(np.radians(shank_angle))
        )

        # Calculate foot points
        shank_rad = np.radians(shank_angle)
        heel = (
            ankle[0] - self.foot_length/2 * np.cos(shank_rad),
            ankle[1] - self.foot_length/2 * np.sin(shank_rad)
        )
        toe = (
            ankle[0] + self.foot_length * np.cos(shank_rad),
            ankle[1] + self.foot_length * np.sin(shank_rad)
        )

        # Add previous positions
        self.line_thigh_prev, = self.ax.plot([hip[0], knee[0]], [hip[1], knee[1]], color=color, alpha=alpha)
        self.line_shank_prev, = self.ax.plot([knee[0], ankle[0]], [knee[1], ankle[1]], color=color, alpha=alpha)
        self.line_foot_prev, = self.ax.plot([toe[0], heel[0]], [toe[1], heel[1]], color=color, alpha=alpha)


    def clear_prev_positions(self):
        # Clear previous positions
        if hasattr(self, 'line_thigh_prev'):
            for line in [self.line_thigh_prev, self.line_shank_prev, self.line_foot_prev]:
                line.set_data([], [])


def plot_stream(max_buffer_size=10, data_name_list = ["ACTUATOR_POSITION", "GRAVITY_VECTOR_Y"]):
    plt.ion()  # Keep interactive mode ON
    
    # Initialize plot once
    
    fig, ax = plt.subplots(len(data_name_list), 1, figsize=(8, 10))
    lines = {name: ax[i].plot([], [], '-o', markersize=3)[0] for i, name in enumerate(data_name_list)}
    
    # Configure plots
    for i, name in enumerate(data_name_list):
        ax[i].set_title(name)
        ax[i].set_xlabel("Time")
        ax[i].set_ylim(-5, 100) if name == "ACTUATOR_POSITION" else ax[i].set_ylim(-10, 10)
    plt.tight_layout()

    data_buffers = {'t': [], **{name: [] for name in data_name_list}}
    frame = 0

    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        buffer = bytearray()
        try:
            while True:
                # 1. Serial Reading (non-blocking)
                while ser.in_waiting > 0:
                    byte = ser.read(1)
                    buffer.extend(byte)
                    
                    # Packet detection
                    if buffer[-1] == START_BYTE and len(buffer) >= PACKET_SIZE:
                        if packet := parse_packet(buffer):
                            frame += 1
                            
                            # Update buffers
                            data_buffers['t'].append(frame)
                            for name in data_name_list:
                                data_buffers[name].append(packet[name])
                            
                            # Trim buffers
                            if len(data_buffers['t']) > max_buffer_size:
                                data_buffers['t'].pop(0)
                                for name in data_name_list:
                                    data_buffers[name].pop(0)
                            
                            # Immediate plot update
                            # for name in data_name_list:
                            for i, name in enumerate(data_name_list):
                                lines[name].set_data(data_buffers['t'], data_buffers[name])
                                ax[i].relim()
                                ax[i].autoscale_view()
                            
                            plt.pause(0.001)  # Micro-pause for GUI
                            
                        buffer = bytearray() # Reset buffer after processing

                # 2. Keep GUI responsive
                plt.pause(0.01)  # Main update interval

        except KeyboardInterrupt:
            plt.ioff()
            plt.show()


def plot_leg_realtime():
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 6))
    plotter = RealTimeLegPlotter(ax)
    
    global reset  # Declare reset as global to avoid UnboundLocalError
    prev_subphase = running_state(5)
    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # visualization:
    
    
    prev_knee_angle = running_state(1)#20)
    prev_shank_angle = running_state(1)#20)
    start_time =  time.time()
    max_knee = running_max()

    # Set up the plot
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        buffer = bytearray()
        program_start_time = time.time()
        try:
            while True:
                if not pause: 
                    
                    if not reset:
                            # clean cache
                        prev_knee_angle.clear()
                        prev_shank_angle.clear()
                        plotter.ax.set_title('Real-Time Leg Sagittal Plane')
                        reset = True
                    
                    # Rapid serial data processing
                    while ser.in_waiting > 0:
                        byte = ser.read(1)
                        buffer.extend(byte)
                        
                        if buffer[-1] == START_BYTE and len(buffer) >= PACKET_SIZE:
                            if packet := parse_packet(buffer):

                                # Calculate angles
                                knee_angle = packet["ACTUATOR_POSITION"] -20
                                shank_angle = get_shank_angle_from_gravity_vec_degree(
                                    packet["GRAVITY_VECTOR_X"],
                                    packet["GRAVITY_VECTOR_Y"]
                                )
                                
                                if  packet["GAIT_SUBPHASE"] > 2 and max_knee.get() < knee_angle:
                                    max_knee.set_data(knee_angle, [knee_angle, shank_angle])
                                    print("max knee angle:", max_knee.get(), "knee angle:", knee_angle, "shank angle:", shank_angle)

                                # testing
                                
                                if time.time() - start_time > 0.1:
                                    print("prev_knee_angle:", prev_knee_angle.data, prev_shank_angle.data)
                                    prev_knee_angle.add(knee_angle)
                                    prev_shank_angle.add(shank_angle)
                                    start_time = time.time()
                                    print("prev_knee_angle:", prev_knee_angle.data, prev_shank_angle.data)

                                plotter.update(knee_angle, shank_angle)
                                plt.pause(0.001)  # Micro-update for smoothness
                                
                                # Update running state
                                prev_subphase.add(packet["GAIT_SUBPHASE"])
                                
                            buffer = bytearray() # Reset buffer after processing
                    
                    plt.pause(0.01)  # Main thread GUI refresh

                else:
                    if not reset:
                        plotter.clear_prev_positions()
                        plotter.ax.set_title("Paused. previous marked positions")
                        for i in range(len(prev_knee_angle.data)):
                            
                            print(f"prev_knee_angle: {prev_knee_angle.data[i]}, prev_shank_angle: {prev_shank_angle.data[i]}")
                            # plotter.add_prev_positions(prev_knee_angle.data[i], prev_shank_angle.data[i], color = 'grey', alpha = 0.5)

                        print(f"max_knee_angle: {max_knee.data[0]}, prev_shank_angle: {max_knee.data[1]}")
                        plotter.add_prev_positions(max_knee.data[0], max_knee.data[1], color = 'grey', alpha = 0.5)
                        # pass
                        reset = True

                        plt.pause(0.1)

                    
                    buffer = bytearray()
       

        except KeyboardInterrupt:
            plt.ioff()
            plt.show()

class running_state:
    # A 1d running state class to manage the running state of the system
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.data = []
        self.max_value = - np.inf

    def add(self, value):
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)

    def average(self):
        return np.mean(self.data) if self.data else 0.0
    
    def min(self):
        return np.min(self.data) if self.data else 0.0
    
    def max(self):
        return np.max(self.data) if self.data else 0.0
    
    def std(self):
        return np.std(self.data) if self.data else 0.0
    
    def clear(self):
        self.data = []

class running_max:
    def __init__(self):
        self.max_value = - np.inf
        self.data = None
    def set_data(self, new_max, value):
        if new_max > self.max_value:
            self.max_value = new_max
            self.data = value
        
    def get(self):
        return self.max_value
    
    



if __name__ == "__main__":
    # plot_stream(max_buffer_size = 8)
    plot_leg_realtime()

