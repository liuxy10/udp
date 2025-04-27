# import json
import matplotlib.pyplot as plt
# import matplotlib
import numpy as np
# from threading import Thread
# from queue import Queue
import serial
from udp_core import SERIAL_PORT, BAUD_RATE, START_BYTE, PACKET_SIZE, SENSOR_DATA, parse_packet, get_shank_angle_from_gravity_vec_degree
# from mpl_toolkits.mplot3d import Axes3D
# import time

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
        ax.axis('equal')
        ax.set_xlim(-0.8, 0.8)
        ax.set_ylim(-0.8, 0.2)
        ax.set_xlabel('X position (m)')
        ax.set_ylabel('Y position (m)')
        ax.set_title('Real-Time Leg Sagittal Plane')
        ax.grid(True)
        ax.legend()

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

    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        buffer = bytearray()
        try:
            while True:
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
                            
                            # Update visualization
                            plotter.update(knee_angle, shank_angle)
                            plt.pause(0.001)  # Micro-update for smoothness
                            
                        buffer = bytearray()
                
                plt.pause(0.01)  # Main thread GUI refresh

        except KeyboardInterrupt:
            plt.ioff()
            plt.show()


if __name__ == "__main__":
    # plot_stream(max_buffer_size = 8)
    plot_leg_realtime()

