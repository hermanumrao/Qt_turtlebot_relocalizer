import time
import csv
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QTextEdit, QPushButton,
    QFileDialog, QTableWidget, QTableWidgetItem, QHeaderView, QHBoxLayout
)
import pyqtgraph as pg


class LoggingTab(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.logs = []

        # Layout setup
        main_layout = QVBoxLayout()
        btn_layout = QHBoxLayout()

        # Buttons
        self.export_btn = QPushButton("Export Logs to CSV")
        self.clear_btn = QPushButton("Clear Logs")
        btn_layout.addWidget(self.export_btn)
        btn_layout.addWidget(self.clear_btn)
        main_layout.addLayout(btn_layout)

        # Table view
        self.table = QTableWidget()
        self.table.setColumnCount(3)
        self.table.setHorizontalHeaderLabels(["Time (s)", "Topic", "Message"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        main_layout.addWidget(self.table)

        # Text view
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        main_layout.addWidget(self.log_display)

        self.setLayout(main_layout)
        self.export_btn.clicked.connect(self.export_logs)
        self.clear_btn.clicked.connect(self.clear_logs)

        # timer bs
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_logs)
        self.timer.start(500)

    def update_logs(self):
        # Appending log and refreshing
        t = round(time.time() - self.ros_node.time_start, 2)
        new_logs = [
            (t, "odom",
             f"Linear: {self.ros_node.linear_speed:.2f}, Angular: {self.ros_node.angular_speed:.2f}"),
            (t, "scan", f"Laser Points: {self.ros_node.laser_points}"),
            (t, "pose", f"Pose Cov Trace: {self.ros_node.pose_cov_trace:.4f}")
        ]
        self.logs.extend(new_logs)
        self.logs = self.logs[-300:]  # 300 lines

        # Table part
        self.table.setRowCount(len(self.logs))
        colors = {"odom": "#03a9f4", "scan": "#4caf50", "pose": "#e91e63"}

        for i, (timestamp, topic, msg) in enumerate(self.logs[-300:]):
            self.table.setItem(i, 0, QTableWidgetItem(f"{timestamp:.2f}"))
            topic_item = QTableWidgetItem(topic)
            topic_item.setForeground(pg.mkColor(colors.get(topic, "#ffffff")))
            self.table.setItem(i, 1, topic_item)
            self.table.setItem(i, 2, QTableWidgetItem(msg))

        # Text View
        html = ""
        for timestamp, topic, msg in self.logs[-50:]:
            html += f"<p style='color:{colors.get(topic, '#fff')};'><b>[{timestamp:6.2f}s] [{topic}]</b> — {msg}</p>"
        self.log_display.setHtml(html)

    def clear_logs(self):
        # Clear all logs
        self.logs.clear()
        self.table.setRowCount(0)
        self.log_display.clear()

    def export_logs(self):
        # Export all logs to CSV
        path, _ = QFileDialog.getSaveFileName(
            self, "Export Logs", "", "CSV Files (*.csv)")
        if path:
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Time (s)", "Topic", "Message"])
                writer.writerows(self.logs)
            print(f"[INFO] Logs exported to {path}")
