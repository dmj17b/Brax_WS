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
import zmq

""" Test Script for sending fake motor data to a ZeroMQ socket for plotting."""

# Create and start the plotting socket
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

def gen_motor_data(socket,start):
    t = time.time()-start
    data_to_send = {
            'time': t,
            'motor1_torque': np.sin(t*10),
        }
    socket.send_pyobj(data_to_send)
    time.sleep(0.002)  # Adjust the sleep time as needed


def main():
    print("Starting the ZeroMQ data sender...")
    start = time.time()
    while True:
        try:
            # This loop will run indefinitely until interrupted
            gen_motor_data(socket,start)

        
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            socket.close()


if __name__ == '__main__':
    main()
