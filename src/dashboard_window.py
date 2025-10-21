import os
import subprocess
import threading
import rclpy
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QTabWidget
from PyQt6.QtCore import Qt

# Import your modular tabs
from ros_node import RosTelemetryNode
from telemetry_tab import TelemetryTab
from logging_tab import LoggingTab
from status_tab import StatusTab


class TurtleBotDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TurtleBot3 Dashboard")
        self.setGeometry(100, 100, 800, 600)

        layout = QVBoxLayout()
        self.setLayout(layout)

        # Tab widget
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        # Control tab
        self.control_tab = QWidget()
        self.status_label = QLabel("Status: random_localizer Stopped")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # Center text
        self.set_status_style("stopped")  # Set initial color

        # Button start stop
        start_btn = QPushButton("Start localizer")
        stop_btn = QPushButton("Stop localizer")
        start_btn.clicked.connect(self.start_pipeline)
        stop_btn.clicked.connect(self.stop_pipeline)

        # control tab
        vbox = QVBoxLayout()
        vbox.addWidget(self.status_label)
        vbox.addWidget(start_btn)
        vbox.addWidget(stop_btn)
        self.control_tab.setLayout(vbox)
        self.tabs.addTab(self.control_tab, "Controls")

        # ROS node
        rclpy.init(args=None)
        self.ros_node = RosTelemetryNode()
        threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True).start()

        # Other tabs
        self.tabs.addTab(TelemetryTab(self.ros_node), "Telemetry")
        self.tabs.addTab(StatusTab(self.ros_node), "Status")
        self.tabs.addTab(LoggingTab(self.ros_node), "Logs")

        # Processes
        self.localizer_process = None
        self.sim_nav_process = None

        # Start Gazebo & rviz2
        self.start_sim_nav()

    # --- Control tab helpers ---
    def set_status_style(self, state):
        """Set status label color based on state."""
        color = {
            "running": "#00c853",  # green
            "stopped": "#f44336",  # red
            "already": "#ffb300"   # orange
        }.get(state, "#ffffff")
        self.status_label.setStyleSheet(f"""
            QLabel {{
                font-size: 14pt;
                font-weight: bold;
                color: {color};
            }}
        """)

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
            self.set_status_style("running")
        else:
            self.status_label.setText("Status: Already Running")
            self.set_status_style("already")

    def stop_pipeline(self):
        if self.localizer_process:
            subprocess.run(["ros2", "launch", "random_localizer", "stop_random_localizer.launch.py"])
            self.localizer_process.wait()
            self.localizer_process = None
            self.status_label.setText("Status: random_localizer Stopped")
            self.set_status_style("stopped")


# --- Main ---
if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    window = TurtleBotDashboard()
    window.show()
    sys.exit(app.exec())

