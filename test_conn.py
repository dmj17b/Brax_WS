import numpy as np
import matplotlib.pyplot as plt
import zmq
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import time
import sys





def plot_data(socket):
    """
    Receive and plot data from ZeroMQ socket.
    """


    while True:
        data_dict = socket.recv_pyobj()
        time = data_dict['time']
        motor1 = data_dict['motor1_torque']
        ax.scatter(time, motor1)
        fig.canvas.draw()
        fig.canvas.flush_events()

        



def main():
    # Set up ZeroMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
        # Set up the plot
    plt.ion()  # Turn on interactive mode
    global fig, ax
    fig, ax = plt.subplots()
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.set_title('Real-Time Data Stream')
    print("Data plotter started. Waiting for data...")
    plot_data(socket)

if __name__ == '__main__':
    main()