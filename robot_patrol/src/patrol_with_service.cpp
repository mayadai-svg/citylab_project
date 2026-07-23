#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node"), has_scan_(false), current_direction_("forward") {
    // Subscriber to store incoming laser scan data
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&Patrol::laser_callback, this, std::placeholders::_1));

    // Velocity publisher to send commands to the robot
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    // Client to communicate with /direction_service
    service_client_ =
        this->create_client<robot_patrol::srv::GetDirection>("/direction_service");

    // 10 Hz control loop timer (runs every 100 ms)
    control_timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::control_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "Client Ready: patrol_with_service client initialized.");
  }

  ~Patrol() {}

private:
  // Class state variables
  sensor_msgs::msg::LaserScan last_laser_scan_;
  bool has_scan_;
  std::string current_direction_;

  // Node publishers, subscribers, clients, and timers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr service_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Store incoming scan data
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    last_laser_scan_ = *laser_msg;
    has_scan_ = true;
  }

  // Helper method to publish twist commands
  void move_robot(double x, double y, double w) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = x;
    msg.linear.y = y;
    msg.angular.z = w;
    vel_publisher_->publish(msg);
  }

  // Periodic 10 Hz control loop
  void control_loop_callback() {
    if (!has_scan_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for laser scan data...");
      return;
    }

    // Call service asynchronously if service is available
    if (service_client_->service_is_ready()) {
      auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
      request->laser_data = last_laser_scan_;

      // LOG MSG: Request Sent
      RCLCPP_INFO(this->get_logger(), "Request Sent to /direction_service.");

      service_client_->async_send_request(
          request,
          std::bind(&Patrol::service_response_callback, this, std::placeholders::_1));
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for /direction_service server...");
    }

    // Compute angular velocity based on latest direction returned by service
    double linear_x = 0.1;
    double angular_z = 0.0;

    if (current_direction_ == "left") {
      angular_z = 0.5;
    } else if (current_direction_ == "right") {
      angular_z = -0.5;
    } else {
      angular_z = 0.0; // "forward"
    }

    // Publish velocity command
    move_robot(linear_x, 0.0, angular_z);
  }

  // Callback to handle service responses
  void service_response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
    try {
      auto response = future.get();
      current_direction_ = response->direction;
      RCLCPP_INFO(this->get_logger(), "Response Received: %s", current_direction_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get response from service: %s", e.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}