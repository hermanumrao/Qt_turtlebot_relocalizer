import sys
import os
import signal
import subprocess
import threading
import time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTabWidget, QDial
)
from pyqtgraph import PlotWidget, plot, mkPen
import pyqtgraph as pg
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import PoseWithCovarianceStamped



import csv
from PyQt6.QtWidgets import QTextEdit, QFileDialog



class RosTelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # Start time for graphs
        self.time_start = time.time()

        # Data for GUI
        self.pose_cov_trace = 0.0
        self.laser_points = 0
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Subscribe to AMCL pose for covariance
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # Subscribe to LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # For speeds, subscribe to /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        # Trace of 6x6 covariance matrix
        self.pose_cov_trace = cov[0] + cov[7] + cov[14] + cov[21] + cov[28] + cov[35]
        # Debug log
        print(f"[Telemetry] Pose covariance trace: {self.pose_cov_trace:.6f}")

    def scan_callback(self, msg: LaserScan):
        
        self.laser_points = sum(1 for r in msg.ranges if 0.0 < r <= 1.0)
        print(f"[Telemetry] Laser points within 1m: {self.laser_points}")

    def odom_callback(self, msg):
        self.linear_speed = msg.twist.twist.linear.x
        self.angular_speed = msg.twist.twist.angular.z
        print(f"[Telemetry] Linear: {self.linear_speed:.2f} m/s, Angular: {self.angular_speed:.2f} rad/s")

class TelemetryTab(QWidget):
    def __init__(self, ros_node: RosTelemetryNode):
        super().__init__()

        self.ros_node = ros_node

        layout = QHBoxLayout()

        # Left: Graphs
        graphs_layout = QVBoxLayout()
        self.pose_graph = PlotWidget(title="Pose Uncertainty vs Time")
        self.pose_graph.setLabel('left', 'Uncertainty')
        self.pose_graph.setLabel('bottom', 'Time', 's')
        self.pose_curve = self.pose_graph.plot([], [], pen=mkPen('r', width=2))
        graphs_layout.addWidget(self.pose_graph)

        self.laser_graph = PlotWidget(title="Laser Points vs Time")
        self.laser_graph.setLabel('left', 'Num Points')
        self.laser_graph.setLabel('bottom', 'Time', 's')
        self.laser_curve = self.laser_graph.plot([], [], pen=mkPen('g', width=2))
        graphs_layout.addWidget(self.laser_graph)

        layout.addLayout(graphs_layout)

        # Right: Centered Speed Dials + Numeric Display
        dials_layout = QVBoxLayout()

        # Linear Speed
        self.linear_display = QLabel("Linear Speed: 0.00 m/s")
        dials_layout.addWidget(self.linear_display)

        self.linear_dial = QDial()
        self.linear_dial.setNotchesVisible(True)
        self.linear_dial.setRange(-200, 200)  # centered dial
        dials_layout.addWidget(self.linear_dial)

        # Angular Speed
        self.angular_display = QLabel("Angular Speed: 0.00 rad/s")
        dials_layout.addWidget(self.angular_display)

        self.angular_dial = QDial()
        self.angular_dial.setNotchesVisible(True)
        self.angular_dial.setRange(-200, 200)
        dials_layout.addWidget(self.angular_dial)

        layout.addLayout(dials_layout)
        self.setLayout(layout)

        # Data buffers for graphs and averaging
        self.time_data = []
        self.pose_data = []
        self.laser_data = []
        self.linear_buffer = []
        self.angular_buffer = []
        self.buffer_time = []

        # Update timer
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(200)  # 5 Hz

    def update_graphs(self):
        t = time.time() - self.ros_node.time_start

        # --- Pose uncertainty ---
        self.time_data.append(t)
        self.pose_data.append(self.ros_node.pose_cov_trace)
        self.laser_data.append(self.ros_node.laser_points)

        # Keep last 200 points
        self.time_data = self.time_data[-200:]
        self.pose_data = self.pose_data[-200:]
        self.laser_data = self.laser_data[-200:]

        self.pose_curve.setData(self.time_data, self.pose_data)
        self.laser_curve.setData(self.time_data, self.laser_data)

        # --- Average speeds over past 2 sec ---
        self.buffer_time.append(t)
        self.linear_buffer.append(self.ros_node.linear_speed)
        self.angular_buffer.append(self.ros_node.angular_speed)

        while self.buffer_time and t - self.buffer_time[0] > 2.0:
            self.buffer_time.pop(0)
            self.linear_buffer.pop(0)
            self.angular_buffer.pop(0)

        avg_linear = sum(self.linear_buffer) / len(self.linear_buffer) if self.linear_buffer else 0.0
        avg_angular = sum(self.angular_buffer) / len(self.angular_buffer) if self.angular_buffer else 0.0

        # Update dials (centered at 0)
        self.linear_dial.setValue(int(avg_linear * 100))
        self.angular_dial.setValue(int(avg_angular * 100))

        # Update numeric displays
        self.linear_display.setText(f"Linear Speed: {avg_linear:.2f} m/s")
        self.angular_display.setText(f"Angular Speed: {avg_angular:.2f} rad/s")

from PyQt6.QtWidgets import QProgressBar


class LoggingTab(QWidget):
    def __init__(self, ros_node: RosTelemetryNode):
        super().__init__()
        self.ros_node = ros_node

        layout = QVBoxLayout()

        # Log viewer
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        layout.addWidget(self.log_display)

        # Export button
        self.export_btn = QPushButton("Export Logs to CSV")
        self.export_btn.clicked.connect(self.export_logs)
        layout.addWidget(self.export_btn)

        self.setLayout(layout)

        # Store logs as tuples (time, topic, message)
        self.logs = []

        # Timer to update logs
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_logs)
        self.timer.start(500)

    def update_logs(self):
        t = round(time.time() - self.ros_node.time_start, 2)

        odom_msg = f"Linear: {self.ros_node.linear_speed:.2f}, Angular: {self.ros_node.angular_speed:.2f}"
        scan_msg = f"Laser Points: {self.ros_node.laser_points}"
        pose_msg = f"Pose Cov Trace: {self.ros_node.pose_cov_trace:.4f}"

        # Append to log buffer
        self.logs.append((t, "odom", odom_msg))
        self.logs.append((t, "scan", scan_msg))
        self.logs.append((t, "pose", pose_msg))

        # Keep only last 300 entries
        self.logs = self.logs[-300:]

        # Display with colors
        html = ""
        for timestamp, topic, msg in self.logs[-50:]:
            color = {
                "odom": "#03a9f4",   # blue
                "scan": "#4caf50",   # green
                "pose": "#e91e63"    # pink
            }.get(topic, "#ffffff")
            html += f"<p style='color:{color};'><b>[{timestamp:6.2f}s] [{topic}]</b> — {msg}</p>"

        self.log_display.setHtml(html)

    def export_logs(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Logs", "", "CSV Files (*.csv)")
        if path:
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Time (s)", "Topic", "Message"])
                writer.writerows(self.logs)
            print(f"[INFO] Logs exported to {path}")



class StatusTab(QWidget):
    def __init__(self, ros_node: RosTelemetryNode):
        super().__init__()
        self.ros_node = ros_node

        layout = QVBoxLayout()

        # Battery display
        self.battery_label = QLabel("Battery Level")
        layout.addWidget(self.battery_label)

        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #555;
                border-radius: 8px;
                text-align: center;
                background-color: #1e1e1e;
                color: white;
            }
            QProgressBar::chunk {
                background-color: #00c853;
                width: 20px;
            }
        """)
        layout.addWidget(self.battery_bar)

        # Velocity displays
        self.velocity_label = QLabel("Current Velocity")
        layout.addWidget(self.velocity_label)

        self.velocity_text = QLabel("Linear: 0.00 m/s | Angular: 0.00 rad/s")
        self.velocity_text.setStyleSheet("font-size: 14pt; color: #e0e0e0;")
        layout.addWidget(self.velocity_text)

        # Timer to update
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)

        self.setLayout(layout)

        # Simulated battery drain parameters
        self.battery_level = 100.0
        self.last_update_time = time.time()

    def update_status(self):
        # Simulate gradual battery drain
        elapsed = time.time() - self.last_update_time
        self.last_update_time = time.time()
        drain_rate = 0.01  # percent per second
        self.battery_level = max(0.0, self.battery_level - drain_rate * elapsed)
        self.battery_bar.setValue(int(self.battery_level))

        # Update velocity readings from ROS node
        linear = self.ros_node.linear_speed
        angular = self.ros_node.angular_speed
        self.velocity_text.setText(f"Linear: {linear:.2f} m/s | Angular: {angular:.2f} rad/s")


class TurtleBotDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TurtleBot3 Dashboard")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.tabs = QTabWidget()
        self.layout.addWidget(self.tabs)

        # Control tab
        self.control_tab = QWidget()
        control_layout = QVBoxLayout()
        self.status_label = QLabel("Status: random_localizer Stopped")
        control_layout.addWidget(self.status_label)

        self.start_btn = QPushButton("Start random_localizer")
        self.start_btn.clicked.connect(self.start_pipeline)
        control_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop random_localizer")
        self.stop_btn.clicked.connect(self.stop_pipeline)
        control_layout.addWidget(self.stop_btn)

        self.control_tab.setLayout(control_layout)
        self.tabs.addTab(self.control_tab, "Controls")

        # Initialize ROS node in background thread
        rclpy.init(args=None)
        self.ros_node = RosTelemetryNode()
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

        # Telemetry tab
        self.telemetry_tab = TelemetryTab(self.ros_node)
        self.tabs.addTab(self.telemetry_tab, "Telemetry")
        
        self.status_tab = StatusTab(self.ros_node)
        self.tabs.addTab(self.status_tab, "Status")
        
        # Logs
        self.logging_tab = LoggingTab(self.ros_node)
        self.tabs.addTab(self.logging_tab, "Logs")



        # Processes
        self.localizer_process = None
        self.sim_nav_process = None

        # Start Gazebo + Nav2 once
        self.start_sim_nav()

    def start_sim_nav(self):
        if self.sim_nav_process is None:
            self.sim_nav_process = subprocess.Popen(
                ["ros2", "launch", "random_localizer", "turtlebot3_sim_nav.launch.py"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            print("[INFO] Started turtlebot3_sim_nav.launch.py")

    def start_pipeline(self):
        if self.localizer_process is None:
            self.localizer_process = subprocess.Popen(
                ["ros2", "launch", "random_localizer", "random_localizer.launch.py"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.status_label.setText("Status: random_localizer Running")
            print("[INFO] Started random_localizer.launch.py")
        else:
            self.status_label.setText("Status: random_localizer Already Running")

    def stop_pipeline(self):
        if self.localizer_process is not None:
            subprocess.run(
                ["ros2", "launch", "random_localizer", "stop_random_localizer.launch.py"]
            )
            self.localizer_process.wait()
            self.localizer_process = None
            self.status_label.setText("Status: random_localizer Stopped")
            print("[INFO] Stopped random_localizer.launch.py")
        else:
            self.status_label.setText("Status: random_localizer Not Running")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TurtleBotDashboard()
    window.show()
    sys.exit(app.exec())

