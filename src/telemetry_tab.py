import time
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDial
import pyqtgraph as pg
from pyqtgraph import PlotWidget, mkPen


class TelemetryTab(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        layout = QHBoxLayout()

        # Graphs, one to check uncertainity and another to check laserscan
        graphs_layout = QVBoxLayout()
        self.pose_graph = PlotWidget(title="Pose Uncertainty vs Time")
        self.pose_curve = self.pose_graph.plot([], [], pen=mkPen('r', width=2))
        graphs_layout.addWidget(self.pose_graph)

        self.laser_graph = PlotWidget(title="Laser Points vs Time")
        self.laser_curve = self.laser_graph.plot(
            [], [], pen=mkPen('g', width=2))
        graphs_layout.addWidget(self.laser_graph)

        layout.addLayout(graphs_layout)

        # Dials to see speed and turns
        dials_layout = QVBoxLayout()
        self.linear_display = QLabel("Linear Speed: 0.00 m/s")
        self.linear_dial = QDial()
        self.linear_dial.setRange(-200, 200)
        dials_layout.addWidget(self.linear_display)
        dials_layout.addWidget(self.linear_dial)

        self.angular_display = QLabel("Angular Speed: 0.00 rad/s")
        self.angular_dial = QDial()
        self.angular_dial.setRange(-200, 200)
        dials_layout.addWidget(self.angular_display)
        dials_layout.addWidget(self.angular_dial)

        layout.addLayout(dials_layout)
        self.setLayout(layout)

        # Buffers
        self.time_data, self.pose_data, self.laser_data = [], [], []
        self.linear_buffer, self.angular_buffer, self.buffer_time = [], [], []

        # Timer for graphing
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(200)

    def update_graphs(self):
        t = time.time() - self.ros_node.time_start

        self.time_data.append(t)
        self.pose_data.append(self.ros_node.pose_cov_trace)
        self.laser_data.append(self.ros_node.laser_points)

        self.time_data, self.pose_data, self.laser_data = self.time_data[-200:
                                                                         ], self.pose_data[-200:], self.laser_data[-200:]

        self.pose_curve.setData(self.time_data, self.pose_data)
        self.laser_curve.setData(self.time_data, self.laser_data)

        self.buffer_time.append(t)
        self.linear_buffer.append(self.ros_node.linear_speed)
        self.angular_buffer.append(self.ros_node.angular_speed)

        while self.buffer_time and t - self.buffer_time[0] > 2.0:
            self.buffer_time.pop(0)
            self.linear_buffer.pop(0)
            self.angular_buffer.pop(0)

        avg_linear = sum(self.linear_buffer) / \
            len(self.linear_buffer) if self.linear_buffer else 0
        avg_angular = sum(self.angular_buffer) / \
            len(self.angular_buffer) if self.angular_buffer else 0

        self.linear_dial.setValue(int(avg_linear * 100))
        self.angular_dial.setValue(int(avg_angular * 100))
        self.linear_display.setText(f"Linear Speed: {avg_linear:.2f} m/s")
        self.angular_display.setText(f"Angular Speed: {avg_angular:.2f} rad/s")
