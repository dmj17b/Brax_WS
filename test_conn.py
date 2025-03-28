import numpy as np
import matplotlib.pyplot as plt
import zmq

def plot_data():
    """
    Receive and plot data from ZeroMQ socket.
    """
    print("Plotting data...")
    # Set up ZeroMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # Set up the plot
    plt.ion()  # Turn on interactive mode
    
    fig, ax = plt.subplots()
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.set_title('Real-Time Data Stream')
    data_dict = socket.recv_pyobj()
    while True:
        data_dict = socket.recv_pyobj()
        time = data_dict['time']
        motor1 = data_dict['motor1_torque']
        ax.scatter(time, motor1)
        fig.canvas.draw()
        fig.canvas.flush_events()

        



def main():
    print("Data plotter started. Waiting for data...")
    plot_data()

if __name__ == '__main__':
    main()