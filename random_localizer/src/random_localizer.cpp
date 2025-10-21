#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class RandomLocalizer : public rclcpp::Node {
public:
  RandomLocalizer() : Node("random_localizer"), obstacle_detected_(false) {
    // Publishers
    initial_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&RandomLocalizer::scanCallback, this, std::placeholders::_1));

    // Publish initial pose after a short delay
    timer_ = this->create_wall_timer(
        2s, std::bind(&RandomLocalizer::publishInitialPose, this));

    // Main loop timer
    move_timer_ = this->create_wall_timer(
        200ms, std::bind(&RandomLocalizer::moveLoop, this));

    RCLCPP_INFO(this->get_logger(), "Random Localizer Node started.");
  }

private:
  void publishInitialPose() {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = 0.0;
    pose.pose.pose.position.y = 0.0;
    pose.pose.pose.orientation.w = 1.0;
    pose.pose.covariance[0] = 0.25;
    pose.pose.covariance[7] = 0.25;
    pose.pose.covariance[35] = 0.0685;

    initial_pose_pub_->publish(pose);
    RCLCPP_INFO(this->get_logger(), "Initial pose published.");
    timer_->cancel(); // only once
  }

	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
		  float min_dist = std::numeric_limits<float>::infinity();

		  // Compute front sector indices
		  double front_angle = M_PI / 12.0; // ±15 degrees in radians
		  int start_idx = std::max(0, int((0.0 - front_angle - msg->angle_min) / msg->angle_increment));
		  int end_idx = std::min(int(msg->ranges.size()) - 1,
		                         int((0.0 + front_angle - msg->angle_min) / msg->angle_increment));

		  for (int i = start_idx; i <= end_idx; i++) {
		      float r = msg->ranges[i];
		      if (std::isfinite(r) && r < min_dist) {
		          min_dist = r;
		      }
		  }

		  obstacle_detected_ = (min_dist < 0.4);
	}


  void moveLoop() {
    geometry_msgs::msg::Twist cmd;
    if (obstacle_detected_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Obstacle detected! Turning...");

      static std::default_random_engine gen(std::random_device{}());
      static std::uniform_real_distribution<double> turn_dist(-1.0, 1.0);

      cmd.angular.z = turn_dist(gen); // Random direction
      cmd.linear.x = 0.0;

      // Turn for a short duration
      turn_counter_++;
      if (turn_counter_ > 30) {
        obstacle_detected_ = false;
        turn_counter_ = 0;
      }
    } else {
      cmd.linear.x = 0.2;
      cmd.angular.z = 0.0;
    }
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr move_timer_;

  bool obstacle_detected_;
  int turn_counter_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomLocalizer>());
  rclcpp::shutdown();
  return 0;
}
