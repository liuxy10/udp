import csv
import socket
import threading
import time


import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np



class DraggablePointsPlot:
    """A draggable points plot for real-time data streaming via UDP. """
    POINT_SELECT_THRESHOLD = 5  # Distance threshold for selecting a point

    def __init__(self, master, xlim=(0,1), ylim=(-20,90), n_points=5, point_select_threshold=None):
        self.master = master
        self.fig, self.ax = plt.subplots(figsize=(5,4))
        self.canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.point_select_threshold = point_select_threshold if point_select_threshold is not None else self.POINT_SELECT_THRESHOLD

        # Initial points
        self.xs = np.array([0, 0.2, 0.50, 0.7, 0.9])
        self.ys = np.array([0, -5, 5, 60, 2])  # Example values, adjust as needed

    
        self.points = self.ax.plot(self.xs, self.ys, 'ro')[0]

        self.ax.set_xlim(*xlim)
        self.ax.set_ylim(*ylim)
        self.ax.set_xlabel("Phase Variable")
        self.ax.set_ylabel("knee angle (degrees)")

        self.canvas.mpl_connect("button_press_event", self.on_press)
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.selected_point = None

        self.update_callback = None  # Set by parent for data update

    def on_press(self, event):
        if event.inaxes != self.ax:
            return
        self.selected_point = None
        min_dist = float('inf')
        for i, (x, y) in enumerate(zip(self.xs, self.ys)):
            dist = np.hypot(x - event.xdata, y - event.ydata)
            if dist < min_dist and dist < self.point_select_threshold:
                min_dist = dist
                self.selected_point = i

    def on_motion(self, event):
        if self.selected_point is None or event.inaxes != self.ax:
            return
        # Clamp to axis limits
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        new_x = min(max(event.xdata, xlim[0]), xlim[1])
        new_y = min(max(event.ydata, ylim[0]), ylim[1])
        self.xs[self.selected_point] = new_x
        self.ys[self.selected_point] = new_y
        self.points.set_data(self.xs, self.ys)
        self.canvas.draw_idle()
        if self.update_callback:
            self.update_callback(self.get_points())  # <-- STREAMS IN REALTIME

    def on_release(self, event):
        self.selected_point = None

    def get_points(self):
        return list(zip(self.xs, self.ys))
    
    def set_update_callback(self, callback):
        self.update_callback = callback

class UDPSender:
    def __init__(self, udp_ip="127.0.0.1", udp_port=6006):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_points(self, points):
        # Send as simple CSV string (e.g. x1,y1;x2,y2;...)
        # Note: x is formatted to 4 decimal places, y to 2 decimal places.
        # Ensure downstream consumers expect this precision.
        data = ';'.join(f"{x:.4f},{y:.2f}" for x, y in points)
        self.sock.sendto(data.encode('utf-8'), (self.udp_ip, self.udp_port))
        # Send as simple CSV string (e.g. x1,y1;x2,y2;...)
        data = ';'.join(f"{x:.4f},{y:.2f}" for x, y in points)
        self.sock.sendto(data.encode('utf-8'), (self.udp_ip, self.udp_port))
        print(f"Sent: {data}")

class FigureSenderGUI:
    def __init__(self, udp_ip="127.0.0.1", udp_port=6006):
        self.root = tk.Tk()
        self.root.title("UDP Sender with Drag Points")
        self.sender = UDPSender(udp_ip, udp_port)

        # Controls
        controls = tk.Frame(self.root)
        controls.pack(side=tk.TOP, fill=tk.X)
        self.status = tk.Label(controls, text="Real-time streaming enabled")
        self.status.pack(side=tk.LEFT, padx=10)

        self.plot = DraggablePointsPlot(self.root)
        self.plot.set_update_callback(self.on_points_updated)
    def on_points_updated(self, points):
        # Whenever a point is dragged, the updated points are sent in real-time via UDP
        # to the configured IP and port, enabling live data streaming to the receiver.
        self.sender.send_points(points)
        self.status.config(text="Streaming data...")
        self.sender.send_points(points)
        self.status.config(text="Streaming data...")

    def start(self):
        self.root.mainloop()

# sender.py
if __name__ == "__main__":

    # Main entry point: start the GUI for real-time UDP point streaming
    gui = FigureSenderGUI()
    gui.start()