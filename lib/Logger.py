import time
import mujoco
import mujoco.viewer
import numpy as np
import yaml
from pathlib import Path
import os
import sys
print(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir,)))
import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl
import AutoSim
import threading
import time
import multiprocessing as mp
import matplotlib
import matplotlib.pyplot as plt


"""Plot real-time data using matplotlib and tkinter"""
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import math
 
 
def SignalSource(amplitude=1, frequency=1):
    """Simulate some signal source that returns an xy data point"""
    start_time = time.time()
    twopi = 2 * math.pi
    period = 1 / frequency
    yield (0, math.sin(0) * amplitude)
    while True:
        x = time.time() - start_time
        phase = x / period * twopi
        yield (x, math.sin(phase) * amplitude)
 
 
class PlotApp(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.signal = SignalSource(frequency=0.5)
        self.xdata = []
        self.ydata = []
 
        # Create a plot that can be embedded in a tkinter window.
        self.figure = Figure(figsize=(6, 4))
        self.plt = self.figure.add_subplot(111)
        canvas = FigureCanvasTkAgg(self.figure, self)
        canvas.draw()
        canvas.get_tk_widget().pack()
        self.update_plot()
 
    def update_plot(self):
        """Get new signal data and update plot.  Called periodically"""
        x, y = next(self.signal)
        self.xdata.append(x)
        self.ydata.append(y)
        if len(self.xdata) > 50:
            # Throw away old signal data
            self.xdata.pop(0)
            self.ydata.pop(0)
        # Refresh plot with new signal data.  Clear the plot so it will rescale to the new xy data.
        self.plt.clear()
        self.plt.margins(x=0)
        self.plt.plot(self.xdata, self.ydata)
        self.figure.canvas.draw()
        self.after(10, self.update_plot)
 
 
PlotApp().mainloop()