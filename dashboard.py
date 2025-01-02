import tkinter as tk
from tkinter import ttk
import serial
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# Establish connection to Arduino
try:
    ser = serial.Serial('COM5', 9600, timeout=1)  # Replace 'COM5' with your port
except Exception as e:
    print(f"Error: {e}")
    ser = None

# Global variables for data
data = []
timestamps = []

def read_arduino():
    """Function to read data from Arduino."""
    while True:
        if ser:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    timestamp = time.time()
                    value = float(line.split(',')[-1])  # Assuming CSV format
                    data.append(value)
                    timestamps.append(timestamp)
                    if len(data) > 100:  # Keep last 100 points
                        data.pop(0)
                        timestamps.pop(0)
                except ValueError:
                    continue
        time.sleep(0.1)

def update_plot():
    """Update the plot with new data."""
    if len(data) > 0:
        ax.clear()
        ax.plot(timestamps, data, label='Distance (cm)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (cm)')
        ax.set_title('Arduino Distance Data')
        ax.legend()
        canvas.draw()
    root.after(500, update_plot)  # Refresh every 500ms

# GUI Setup
root = tk.Tk()
root.title("Arduino Distance Sensor GUI")

frame = ttk.Frame(root)
frame.pack(padx=10, pady=10)

# Matplotlib Figure
fig, ax = plt.subplots(figsize=(6, 4))
canvas = FigureCanvasTkAgg(fig, master=frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack()

# Thread for Arduino data
thread = threading.Thread(target=read_arduino, daemon=True)
thread.start()

# Start updating the plot
update_plot()

# Run the GUI
root.mainloop()
