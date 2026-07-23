#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

using namespace std::chrono_literals;

class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service_node"), has_laser_scan_(false) {
    // Subscriber to store the latest laser scan from fastbot_1
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", 10,
        std::bind(&TestService::laser_callback, this, std::placeholders::_1));

    // Service Client for /direction_service
    service_client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");

    // Timer running every 1 second to request direction from the service
    timer_ = this->create_wall_timer(
        1s, std::bind(&TestService::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "TestService node initialized.");
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store latest scan and set flag
    last_laser_scan_ = *msg;
    has_laser_scan_ = true;
  }

  void timer_callback() {
    if (!has_laser_scan_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for first laser scan data...");
      return;
    }

    if (!service_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Service [/direction_service] is not available yet.");
      return;
    }

    // Create the service request using stored laser scan
    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = last_laser_scan_;

    // Send request asynchronously
    service_client_->async_send_request(
        request,
        std::bind(&TestService::response_callback, this, std::placeholders::_1));
  }

  void response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
    try {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Service Response Direction: %s", response->direction.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get service response: %s", e.what());
    }
  }

  // Member variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr service_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan last_laser_scan_;
  bool has_laser_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestService>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}