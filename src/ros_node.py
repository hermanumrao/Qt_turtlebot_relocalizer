import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped


class RosTelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        self.time_start = time.time()
        self.pose_cov_trace = 0.0
        self.laser_points = 0
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
    # in small lines i use trace of covariance matrix to calculate uncertainnity in pose

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        self.pose_cov_trace = cov[0] + cov[7] + \
            cov[14] + cov[21] + cov[28] + cov[35]
        print(f"[Telemetry] Pose covariance trace: {self.pose_cov_trace:.6f}")

    def scan_callback(self, msg: LaserScan):
        # i check for number of points withing 1m radius
        self.laser_points = sum(1 for r in msg.ranges if 0.0 < r <= 1.0)
        print(f"[Telemetry] Laser points within 1m: {self.laser_points}")

    def odom_callback(self, msg: Odometry):
        # this basically gets me odom data to know current linear and angular speeds
        self.linear_speed = msg.twist.twist.linear.x
        self.angular_speed = msg.twist.twist.angular.z
        print(
            f"[Telemetry] Linear: {self.linear_speed:.2f}, Angular: {self.angular_speed:.2f}")
