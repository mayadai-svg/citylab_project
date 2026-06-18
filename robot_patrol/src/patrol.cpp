#include <memory>
#include <cmath>
#include <string>
#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
    odom_subscription_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastbot_1/odom", 10,
            std::bind(&Patrol::odom_callback, this, std::placeholders::_1));
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Patrol::control_loop_callback, this));
  }

  // class destructor
  ~Patrol() {
    // write your termination codes here if any
  }

private:
  // variables
  double direction_ = 0.0; // Initialize direction_ to 0.0 radians (facing forward)
  double obstacle_threshold = 0.35; // 35 cm
  int obstacle_detection = 0; // Tracks the number of blocked sections

  // publishers & subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    // RCLCPP_INFO(this->get_logger(), "Reading laser_scan messages");

    int laserscan_size = static_cast<int>(laser_msg->ranges.size());
    if (laserscan_size == 0 || laser_msg->angle_increment == 0.0) {
      return;
    }

    // Reset detection counter at the start of a new scan
    obstacle_detection = 0;

    // define sectors using ray angles
    std::map<std::string, std::pair<double, double>> sectors = {
      {"right", {-M_PI/2, -M_PI/12}},
      {"front", {-M_PI/12, M_PI/12}},
      {"left", {M_PI/12, M_PI/2}}};

    // initialize min_distances map
    std::map<std::string, double> min_distances;

    // --- PRIORITY 1: Scan Front Sector ---
    double front_min = std::numeric_limits<double>::infinity(); // initialize to infinity for min comparison
    for (size_t i = 0; i < laser_msg->ranges.size(); ++i) {
      double angle = laser_msg->angle_min + (i * laser_msg->angle_increment);
      if (angle >= sectors["front"].first && angle <= sectors["front"].second) {
        if (laser_msg->ranges[i] < front_min) front_min = laser_msg->ranges[i];
      }
    }

    // Check Front Threshold
    if (front_min < obstacle_threshold) {
      min_distances["front"] = front_min;
      obstacle_detection++; // Front is blocked, increment obstacle detection count

      // --- PRIORITY 2: Scan Right Sector (Only because Front is blocked) ---
      double right_min = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < laser_msg->ranges.size(); ++i) {
        double angle = laser_msg->angle_min + (i * laser_msg->angle_increment);
        if (angle >= sectors["right"].first && angle <= sectors["right"].second) {
          if (laser_msg->ranges[i] < right_min) right_min = laser_msg->ranges[i];
        }
      }

      // Check Right Threshold
      if (right_min < obstacle_threshold) {
        min_distances["right"] = right_min;
        obstacle_detection++; // Right is also blocked, increment obstacle detection count

        // --- PRIORITY 3: Scan Left Sector (Only because Front AND Right are blocked) ---
        double left_min = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < laser_msg->ranges.size(); ++i) {
          double angle = laser_msg->angle_min + (i * laser_msg->angle_increment);
          if (angle >= sectors["left"].first && angle <= sectors["left"].second) {
            if (laser_msg->ranges[i] < left_min) left_min = laser_msg->ranges[i];
          }
        }

        // Check Left Threshold
        if (left_min < obstacle_threshold) {
          min_distances["left"] = left_min;
          obstacle_detection++; // All three are blocked
        }
      }
    }

    // Run processing logic to calculate direction_
    safest_direction(laser_msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    // Unused placeholder for the odom subscription initialization in constructor
    (void)odom_msg;
  }

  void move_robot(double x, double y, double w) {
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

  void stop_robot() {
    move_robot(0, 0, 0); // Stop the robot by setting all velocities to zero
  }

  void safest_direction(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    // CASE 0: Clear path ahead
    if (obstacle_detection == 0) {
      direction_ = 0.0;
      return;
    }

    // CASE 1: Front blocked -> Instantly choose Right sector center
    if (obstacle_detection == 1) {
      direction_ = (-M_PI / 2.0 + -M_PI / 12.0) / 2.0; 
      return;
    } 

    // CASE 2: Front & Right blocked -> Instantly choose Left sector center
    if (obstacle_detection == 2) {
      direction_ = (M_PI / 12.0 + M_PI / 2.0) / 2.0; 
      return;
    }

    // CASE 3: All standard sectors blocked! 
    // NOW the loop activates to find the "least worst" escape ray out of everything.
    double max_distance = -1.0;
    double safest_ray_angle = 0.0;

    for (size_t i = 0; i < laser_msg->ranges.size(); ++i) {
      double angle = laser_msg->angle_min + (i * laser_msg->angle_increment);
      
      // Only look within our valid steering field of view
      if (angle >= -M_PI / 2.0 && angle <= M_PI / 2.0) {
        double current_range = laser_msg->ranges[i];
        
        if (std::isfinite(current_range) && current_range > max_distance) {
          max_distance = current_range;
          safest_ray_angle = angle;
        }
      }
    }

    // Steer toward the center of whatever sector held that "least worst" ray
    if (safest_ray_angle < -M_PI / 12.0) {
      direction_ = (-M_PI / 2.0 + -M_PI / 12.0) / 2.0; // Right Center
    } else if (safest_ray_angle > M_PI / 12.0) {
      direction_ = (M_PI / 12.0 + M_PI / 2.0) / 2.0;  // Left Center
    } else {
      direction_ = 0.0;                                // Front Center
    }
  }

  // Create a control callback loop that runs at 10 Hz.
  void control_loop_callback() {
    // At every iteration of the control loop, publish the proper velocity command to the topic, based on the values detected by the laser.
    // The linear velocity in X must always be 0.1 m/s.
    // Take the value of the variable direction_ and use it to compute the proper angular velocity in Z:
    // angular velocity in z = direction_ / 2.
    
    double linear_x = 0.1;
    double angular_z = direction_ / 2.0;
    
    move_robot(linear_x, 0.0, angular_z);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}