#include <cmath>
#include <limits>
#include <map>
#include <memory>

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

// 1- Create C++ class called Patrol
// 2- Subscribe to laser topic and capture rays
// 3- Only use rays corresponding to the front 180 degrees
// 4- Implement patrol algorithm
// 4a- Move robot forward until an obstacle is detected in front of the robot
// closer than 35cm
// 4b- When the robot detects an obstacle in front, apply the
// following logic in order to avoid it:
// - Get the rays corresponding to those 180º.
// - Identify the ray with the greatest distance (which is not an
// inf) Get its corresponding angle with respect to the robot's front X axis
// - Note: The angle must be between  −𝜋/2 and  𝜋/2 (in radians).
// - Store that angle in a class variable named direction_.
// - This angle represents the direction of the most open area, and therefore
// the direction the robot should rotate toward.
// 5- Move robot to safest area
// To move the robot toward the safe direction, you need to publish the
// appropriate values to the robot’s velocity topic.
// 5a- Inside the patrol.cpp, create a publisher to the /cmd_vel topic.
// 5b- Create a control callback loop that runs at 10 Hz.
// 5c- At every iteration of the control loop, publish the
// proper velocity command to the topic, based on the values detected by the
// laser. The linear velocity in X must always be 0.1 m/s. Take the value of the
// variable direction_ and use it to compute the proper angular velocity in Z:
// angular velocity in z = direction_ / 2.

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);
  }

  // class destructor
  ~Patrol() {
    // write your termination codes here if any
  }

private:
  // variables
  double direction_ =
      0.0; // Initialize direction_ to 0.0 radians (facing forward)

  // publishers & subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    RCLCPP_INFO(this->get_logger(), "Reading laser_scan messages");

    // Define sectors and minimum distances for obstacle detection
    std::map<std::string, std::pair<int, int>> sectors = {
        {"right", {37, 62}},
        {"front_right", {63, 99}},
        {"front_left", {100, 137}},
        {"left", {133, 162}}};

    std::map<std::string, double> min_distances;
    for (const auto &sector : sectors) {
      double min_distance = std::numeric_limits<double>::infinity();
      for (int i = sector.second.first; i <= sector.second.second; ++i) {
        if (laser_msg->ranges[i] < min_distance) {
          min_distance = laser_msg->ranges[i];
        }
      }
      min_distances[sector.first] = min_distance;
    }

    double obstacle_threshold = 0.35; // 35 cm

    std::map<std::string, bool> detections;
    for (const auto &[sector, min_distance] : min_distances) {
      if (min_distance < obstacle_threshold) {
        detections[sector] = min_distance < obstacle_threshold;
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "Obstacle detections: right: %s, front_right: %s, front_left: "
                "%s, left: %s",
                detections["right"] ? "true" : "false",
                detections["front_right"] ? "true" : "false",
                detections["front_left"] ? "true" : "false",
                detections["left"] ? "true" : "false");

    safest_direction(laser_msg);
    double angular_velocity = direction_ / 2.0;
    move_rover(0.1, 0.0, angular_velocity);
  }

  void move_rover(double x, double y, double w) {
    // Here you have the callback method
    // create a Twist message
    auto msg = geometry_msgs::msg::Twist();
    // define the linear x-axis velocity of /cmd_vel Topic parameter
    msg.linear.x = x;
    msg.linear.y = y;
    // define the angular z-axis velocity of /cmd_vel Topic parameter
    msg.angular.z = w;
    // Publish the message to the Topic
    vel_publisher_->publish(msg);
  }

  void
  safest_direction(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    // Implement logic to find the safest direction based on laser scan data
    // 1- Identify the ray with the greatest distance to an obstacle (which is
    // not an inf) 2- Get its corresponding angle with respect to the robot's
    // front X axis Note: The angle must be between  −𝜋/2 and  𝜋/2 (in radians)
    // 3- Store that angle in the class variable named direction_

    if (laser_msg->ranges.empty()) {
      direction_ = 0.0;
      return;
    }

    size_t best_ray_index = 0;
    double best_valid_range = -1.0;
    for (size_t i = 0; i < laser_msg->ranges.size(); ++i) {
      double range = laser_msg->ranges[i];
      if (!std::isfinite(range)) {
        continue;
      }
      if (range > best_valid_range) {
        best_valid_range = range;
        best_ray_index = i;
      }
    }

    if (best_valid_range < 0.0) {
      direction_ = 0.0;
      return;
    }

    double angle =
        laser_msg->angle_min + best_ray_index * laser_msg->angle_increment;
    double half_pi = std::acos(-1.0) / 2.0;
    if (angle < -half_pi) {
      angle = -half_pi;
    } else if (angle > half_pi) {
      angle = half_pi;
    }

    direction_ = angle;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}