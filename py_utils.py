import json
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
from queue import Queue
import serial
from udp_core import SERIAL_PORT, BAUD_RATE, START_BYTE, PACKET_SIZE, SENSOR_DATA, parse_packet
from mpl_toolkits.mplot3d import Axes3D
import time
def get_json_data(path):
    with open(path) as f:
        data = json.load(f)
    return data

def save_json_data(path, data):
    with open(path, 'w') as f:
        json.dump(data, f, indent=4)
    
def plot_data(max_buffer_size = 5):
    global queue  # Use the global queue
    queue = Queue(maxsize=max_buffer_size)  # Create a queue with a maximum size
    start_rendering(queue, max_buffer_size = 20) 

    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        buffer = bytearray()
        try:
            while True:
                byte = ser.read(1)
                if not byte:
                    continue

                buffer.extend(byte)

                # If a start byte is found and we have a full packet, process it.
                if buffer[-1] == START_BYTE and len(buffer) >= PACKET_SIZE:
                    packet = parse_packet(buffer)
                    if packet:
                        # print (', '.join(f"{packet[name]:.4f}" for name, _ in SENSOR_DATA))
                        queue.put([packet[name] for name, _ in SENSOR_DATA])  # Add the parsed packet to the queue
                        # print(len(queue))
                        # print(len(queue.queue)) if not queue.empty() else print("Queue is empty")
                        if queue.full():
                            # render_raw_data(queue, max_buffer_size)  # Call the rendering function
                            # pop the oldest data from the queue
                            queue.get()
                            
                            # time.sleep(0.03)
                    
                    buffer = bytearray()  # Reset the buffer
                    

        except KeyboardInterrupt:
            print("\nRendering stopped.")
            queue.put(None)  # Signal the rendering thread to stop
def render_raw_data(queue, max_buffer_size=5):
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(3, 1, figsize=(8, 10))

    # Initialize empty plots
    lines = {
        'gait_subphase': ax[0].plot([], [], 'g-')[0],
        'actuator_position': ax[1].plot([], [], 'b-')[0],
        'gravity_vector_x': ax[2].plot([], [], 'c-')[0],
    }
    inds = {key: idx for idx, (key, _) in enumerate(SENSOR_DATA)}

    # Set titles and y-axis limits for each subplot
    ax[0].set_title("Gait Subphase")
    ax[0].set_ylim(0, 5)
    ax[1].set_title("knee angle")
    ax[1].set_ylim(-5, 100)
    ax[2].set_title("Gravity Vector (X)")
    ax[2].set_ylim(-10, 10)

    # Set x-axis labels for each subplot
    for axis in ax:
        axis.set_xlabel("Time")

    plt.tight_layout()

    # Data buffers
    data_buffers = {key: [] for key in lines.keys()}  # (time, value)
    data_buffers['t'] = []
    frame = 0
    while True:
        # try:
        data_line = queue.get(timeout=0.3)  # Wait for new data
        # print(data_line, len(data_buffers))
        if data_line is None:  # Exit signal
            break
        
        # Update data buffers
        for key in lines.keys():
            if key != "t":
                data_buffers[key].append(data_line[inds[key.upper()]])  # Get the corresponding value from the data line

        # Update time buffer
        data_buffers['t'].append(frame)  # Add the current frame as the time value

        # Update plots
        for key, line in lines.items():
            if key != "t" and len(data_buffers['t']) == len(data_buffers[key]):  # Ensure lengths match
                line.set_xdata(data_buffers['t'])  # Set x data to time
                line.set_ydata(data_buffers[key])
        for i in range(3):
            if len(data_buffers['t']) > 0:  # Ensure there is data to relim and autoscale
                ax[i].relim()  # Recalculate limits for the axis
                ax[i].autoscale_view()  # Autoscale the view to fit the data

        plt.pause(0.01)  # Allow the plot to update

        # Limit buffer size
        for key in data_buffers.keys():
            if len(data_buffers[key]) > max_buffer_size:
                data_buffers[key].pop(0)

        frame += 1
        # except Exception as e:
        #     print(f"Error in rendering data: {e}")
        #     continue

    plt.ioff()  # Turn off interactive mode
    plt.show()

def start_rendering(queue, max_buffer_size=5):
    render_thread = Thread(target=render_raw_data, args=(queue, max_buffer_size,))
    
    # render_thread = Thread(target=render_frame_live, args=(queue,))
    render_thread.daemon = True # Make the thread a daemon thread, a daemon thread will exit when the main program exits
    render_thread.start()
    return render_thread




def render_frame_live(queue):
    # render 3d plot showing the knee's orientation
    plt.ion()  # Turn on interactive mode
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_title("Knee Orientation")
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")

    # Initialize a point in 3D space
    point, = ax.plot([0], [0], [0], 'ro')  # Red dot for the knee orientation
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])
    while True:
        try:
            data_line = queue.get(timeout=0.05)  # Wait for new data
            if data_line is None:  # Exit signal
                break
            # Update the point's position based on gravity vector
            point.set_data([data_line['GRAVITY_VECTOR_X']], [data_line['GRAVITY_VECTOR_Y']])
            point.set_3d_properties([data_line['GRAVITY_VECTOR_Z']])

            plt.pause(0.01)  # Allow the plot to update
        except Exception:
            continue

    plt.ioff()  # Turn off interactive mode
    plt.show()
    

if __name__ == "__main__":
    plot_data(max_buffer_size = 10)

