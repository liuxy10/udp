import socket
import threading
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

class UDPReceiverGUI:
    def __init__(self, udp_params1=('127.0.0.1', 5005), udp_params2=('127.0.0.1', 6006)):
        self.udp_params1 = udp_params1
        self.udp_params2 = udp_params2
        self.root = tk.Tk()
        self.root.title("Dual UDP Receiver")

        frame1 = tk.Frame(self.root)
        frame1.pack(side=tk.LEFT, padx=10, pady=10)
        tk.Label(frame1, text=f"Receiver 1: {udp_params1}").pack()
        self.text_area1 = ScrolledText(frame1, wrap=tk.WORD, width=40, height=20)
        self.text_area1.pack()

        frame2 = tk.Frame(self.root)
        frame2.pack(side=tk.RIGHT, padx=10, pady=10)
        tk.Label(frame2, text=f"Receiver 2: {udp_params2}").pack()
        self.text_area2 = ScrolledText(frame2, wrap=tk.WORD, width=40, height=20)
        self.text_area2.pack()

        # Start the two receiver threads
        threading.Thread(target=self.listen_udp, args=(self.udp_params1, self.text_area1), daemon=True).start()
        threading.Thread(target=self.listen_udp, args=(self.udp_params2, self.text_area2), daemon=True).start()

    def listen_udp(self, udp_params, text_area):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(udp_params)
        while True:
            data, addr = sock.recvfrom(1024)
            message = f"{addr}: {data.decode('utf-8')}"
            # Make sure GUI updates run on the main thread
            text_area.after(0, self.display_message, text_area, message)

    def display_message(self, text_area, msg):
        text_area.insert(tk.END, msg + "\n")
        text_area.see(tk.END)

    def start(self):
        self.root.mainloop()

# Example usage
if __name__ == "__main__":
    gui = UDPReceiverGUI(
        udp_params1=('127.0.0.1', 5005),
        udp_params2=('127.0.0.1', 6006)
    )
    gui.start()
