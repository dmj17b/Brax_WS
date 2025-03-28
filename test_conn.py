import numpy as np
import matplotlib.pyplot as plt
import zmq
from pyqtgraph.Qt import QtCore, QtGui
from PyQt6.QtWidgets import QApplication, QWidget

import pyqtgraph as pg
import time
import sys





def plot_data(socket,plt):
    """
    Receive and plot data from ZeroMQ socket.
    """


    while True:
        data_dict = socket.recv_pyobj()
        time = data_dict['time']
        motor1 = data_dict['motor1_torque']
        print(f"Received data: Time={time}, Motor1 Torque={motor1}")
        plt.scatterPlot(point = [time,motor1], pen='b', symbol='o')  # Plot the received data

        



def main():
    # Set up ZeroMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # Setting up pyqtgraph for real-time plotting
    app = QApplication([])  # Create a Qt application
    grph = pg.PlotWidget(title="Real-time Motor Data Plotter")
    

    print("Data plotter started. Waiting for data...")
    plot_data(socket,grph)

if __name__ == '__main__':
    main()