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

class DataSender():
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")

        self.log_interval = 0.05
        self.last_log_time = 0

    def send_data(self, sim_time):
        if(sim_time-self.last_log_time>=self.log_interval):
            self.data_to_send = {
                'time': sim_time,
                'motor1_torque': np.sin(sim_time*2),
                'motor2_torque': np.cos(sim_time*5),
                'motor3_torque': np.cos(sim_time*10),
            }
            print(sim_time)
            self.socket.send_pyobj(self.data_to_send)
            self.last_log_time = sim_time


def main():
    logger = DataSender()
    print("Starting the data sender...")
    start = time.time()
    logger.last_log_time = time.time()-start
    while True:
        try:
            # This loop will run indefinitely until interrupted
            sim_time = time.time()-start
            logger.send_data(sim_time)

        except KeyboardInterrupt:
            print("Keyboard interrupt")
            logger.socket.close()


if __name__ == '__main__':
    main()
