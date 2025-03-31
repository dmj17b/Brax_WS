import sys
import pyqtgraph as pg
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout
import zmq
import numpy as np

class ScatterPlotApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        # Set up ZeroMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(1)


    def initUI(self):
        self.setWindowTitle("PyQt Scatter Plot Example")
        self.setGeometry(100, 100, 800, 600)

        self.plot_widget = pg.PlotWidget()
        self.scatter_plot = pg.ScatterPlotItem()
        # Set the y limit
        self.scatter_plot.yRange = 1.0
        self.plot_widget.addItem(self.scatter_plot)

        layout = QVBoxLayout(self)
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

        self.show()

    def plot_data(self, x_data, y_data):
         self.scatter_plot.setData(x=x_data, y=y_data, symbol='o', size=10, brush='r')

    def update_plot(self):
        data_dict = self.socket.recv_pyobj()
        time = np.array([data_dict['time']])
        motor1_torque = np.array([data_dict['motor1_torque']])
        self.plot_data(time, motor1_torque)
        # print(f"Received data: Time={time}, Motor1 Torque={motor1_torque}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    scatter_app = ScatterPlotApp()
    
    sys.exit(app.exec())