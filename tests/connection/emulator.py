import csv
import socket
import threading
import time

import json
import numpy as np
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
# data structure for the feature data, when sent, use json so that it can be easily named and parsed:
feature_data = {
    # meta data
    "gait_count": 0,
    "step_length": 0,

    # feature data
    "initial_knee_angle": 0

}

# Test class for emulating asynchronous feature data sending over UDP

# Test class for emulating CSV stream data sending over UDP
class CSVEmulatorUDPSender:
    def __init__(self, csv_path, udp_ip='127.0.0.1', udp_port=5005, interval=0.02):
        self.csv_path = csv_path
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.interval = interval
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()

    def send_emulated_data(self):
        with open(self.csv_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if self._stop.is_set():
                    break
                message = ','.join(row)
                self.sock.sendto(message.encode('utf-8'), (self.udp_ip, self.udp_port))
                print(f"Sent: {message}")
                time.sleep(self.interval)
        # Socket will be closed in stop()

    def start(self):
        self._stop.clear()
        threading.Thread(target=self.send_emulated_data, daemon=True).start()

    def stop(self):
        self._stop.set()

if __name__ == "__main__":
    # sender = CSVEmulatorUDPSender('000_log_20250730_114148.csv')
    # sender.start()
    # # Let it run; in production, wire up to a button, signal, etc.
    # time.sleep(100)  # Example: run for 10 seconds
    # sender.stop()
    pass