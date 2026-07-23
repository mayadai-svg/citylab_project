#include <memory>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service") {
    // Create the service server advertising on '/direction_service'
    service_ = this->create_service<robot_patrol::srv::GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::direction_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service Server Ready: [/direction_service] is online.");  
  }

private:
  void direction_callback(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {
    
    // LOG MSG: Request Received
    RCLCPP_INFO(this->get_logger(), "Request Received on /direction_service.");

    // Access laser scan data passed in request->laser_data
    const auto &laser_data = request->laser_data;

    // Guard against empty scan messages
    if (laser_data.ranges.empty() || laser_data.angle_increment == 0.0) {
      RCLCPP_WARN(this->get_logger(), "Received empty or invalid laser scan!");
      response->direction = "forward";
      RCLCPP_INFO(this->get_logger(), "Request Completed with default direction: forward");
      return;
    }

    double total_dist_sec_right = 0.0;
    double total_dist_sec_front = 0.0;
    double total_dist_sec_left = 0.0;
    double front_min = std::numeric_limits<double>::infinity();

    // Define 3 sections of 60 degrees each (-90 deg to +90 deg total FOV):
    // Right: [-90°, -30°] -> [-M_PI/2, -M_PI/6]
    // Front: [-30°,  30°] -> [-M_PI/6,  M_PI/6]
    // Left:  [ 30°,  90°] -> [ M_PI/6,  M_PI/2]

    for (size_t i = 0; i < laser_data.ranges.size(); ++i) {
      double angle = laser_data.angle_min + (i * laser_data.angle_increment);
      double range = laser_data.ranges[i];

      // Ignore invalid readings (NaN, Inf, or out-of-bounds)
      if (!std::isfinite(range) || range < laser_data.range_min || range > laser_data.range_max) {
        continue;
      }

      // 1) Right sector (Blue): -90° to -30° [-PI/2, -PI/6]
      if (angle >= -M_PI / 2.0 && angle < -M_PI / 6.0) {
        total_dist_sec_right += range;
      } 
      // 2) Front sector (Green): -30° to +30° [-PI/6, PI/6]
      else if (angle >= -M_PI / 6.0 && angle <= M_PI / 6.0) {
        total_dist_sec_front += range;
        if (range < front_min) {
        front_min = range;
        }
      } 
      // 3) Left sector (Red): +30° to +90° [PI/6, PI/2]
      else if (angle > M_PI / 6.0 && angle <= M_PI / 2.0) {
        total_dist_sec_left += range;
      }
    }

    RCLCPP_INFO(this->get_logger(),
                    "Request Completed. Direction: %s | Front Min: %.2f m | Total Front: %.2f m | Total Left: %.2f m | Total Right: %.2f m",
                    front_min, total_dist_sec_front, total_dist_sec_left, total_dist_sec_right);    
    
    // Set response direction string (e.g., "forward", "left", "right")
    // Decision Logic:
    // 1. If minimum distance in front section > 0.35m -> Move forward
    if (front_min > 0.35) {
      response->direction = "forward";
    } 
    // 2. Otherwise, turn towards the side with the larger total distance sum
    else {
      if (total_dist_sec_left > total_dist_sec_right) {
        response->direction = "left";
      } else {
        response->direction = "right";
      }
    }

    RCLCPP_INFO(this->get_logger(), "Selected safest direction: %s", response->direction.c_str());
  }

  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}