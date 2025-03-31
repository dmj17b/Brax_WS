import sys
import pyqtgraph as pg
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout
import zmq
import numpy as np
import time

class SpeedTorquePlotter(QWidget):
    def __init__(self, num_motors):
        super().__init__()
        # Set up ZeroMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(0)
        self.num_motors = num_motors
        
        # Store historical data
        self.historical_data = [{'time': [], 'torque': []} for _ in range(self.num_motors)]
        # Maximum number of points to keep (to prevent memory issues)
        self.max_points = 100  # Adjust this based on your needs
        
        self.initUI()

    def initUI(self):
        self.setWindowTitle("PyQt Scatter Plot with History")
        self.setGeometry(100, 100, 800, 600)

        # Initializing plot widgets and plots
        self.plot_widgets = []
        self.plots = []
        layout = QVBoxLayout(self)

        # Initialize the plot widgets and plots
        for i in range(self.num_motors):
            plot_widget = pg.PlotWidget()
            plot_widget.setLabel('left', f'Motor {i+1} Torque')
            plot_widget.setLabel('bottom', 'Time')
            
            # Use PlotDataItem instead of ScatterPlotItem for better performance with many points
            plot = plot_widget.plot(pen=None, symbol='o', symbolSize=5, symbolBrush='r')
            
            self.plot_widgets.append(plot_widget)
            self.plots.append(plot)
            layout.addWidget(plot_widget)
        
        self.setLayout(layout)
        self.show()

    def update_plot(self):
        # If no data is received, return
        if self.socket.poll(0) == 0:
            return

        data_dict = self.get_data()
        t = data_dict['time']
        
        for i in range(self.num_motors):
            motor_key = f'motor{i+1}_torque'
            if motor_key in data_dict:
                motor_torque = data_dict[motor_key]
                
                # Append new data to historical data
                self.historical_data[i]['time'].append(t)
                self.historical_data[i]['torque'].append(motor_torque)
                
                # Limit the number of points to prevent memory issues
                if len(self.historical_data[i]['time']) > self.max_points:
                    self.historical_data[i]['time'] = self.historical_data[i]['time'][-self.max_points:]
                    self.historical_data[i]['torque'] = self.historical_data[i]['torque'][-self.max_points:]
                
                # Update the plot with all historical data
                self.plots[i].setData(
                    x=self.historical_data[i]['time'], 
                    y=self.historical_data[i]['torque']
                )

    def get_data(self):
        data = self.socket.recv_pyobj()
        return data

if __name__ == '__main__':
    app = QApplication(sys.argv)
    scatter_app = SpeedTorquePlotter(num_motors=3)
    sys.exit(app.exec())