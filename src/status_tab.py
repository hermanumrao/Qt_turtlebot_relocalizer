import time
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QProgressBar
import pyqtgraph as pg


class StatusTab(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        layout = QVBoxLayout()
        # hadling battery levels (it isn't simulated, i have put some random stuff)
        self.battery_label = QLabel("Battery Level")
        layout.addWidget(self.battery_label)

        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(100)
        layout.addWidget(self.battery_bar)
        # displaying raw speed and agular vel
        self.velocity_label = QLabel("Current Velocity")
        self.velocity_text = QLabel("Linear: 0.00 m/s | Angular: 0.00 rad/s")
        layout.addWidget(self.velocity_label)
        layout.addWidget(self.velocity_text)

        self.setLayout(layout)
        self.battery_level = 100.0
        self.last_update = time.time()

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)

    def update_status(self):
        elapsed = time.time() - self.last_update
        self.last_update = time.time()
        self.battery_level = max(0.0, self.battery_level - 0.01 * elapsed)
        self.battery_bar.setValue(int(self.battery_level))

        self.velocity_text.setText(
            f"Linear: {self.ros_node.linear_speed:.2f} m/s | Angular: {self.ros_node.angular_speed:.2f} rad/s"
        )
