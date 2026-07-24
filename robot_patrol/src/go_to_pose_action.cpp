#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "robot_patrol/action/go_to_pose.hpp"

using namespace std::chrono_literals;

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  GoToPose() : Node("go_to_pose_action_server"), has_odom_(false) {
    // 1. Action Server named /go_to_pose
    action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "/go_to_pose",
        std::bind(&GoToPose::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&GoToPose::handle_cancel, this, std::placeholders::_1),
        std::bind(&GoToPose::handle_accepted, this, std::placeholders::_1));

    // 2. Subscribe to odometry topic (/fastbot_1/odom)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&GoToPose::odom_callback, this, std::placeholders::_1));

    // 3. Publisher to velocity command topic (/fastbot_1/cmd_vel)
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Action Server Ready: /go_to_pose initialized.");
  }

private:
  // Class state variables
  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;
  bool has_odom_;

  // Node handles
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // Normalize angle to [-pi, pi] range
  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // 2. Odometry Callback: Extract (x, y) and convert Quaternion to Euler Yaw (theta)
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    // Convert orientation quaternion to Euler angles
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pos_.theta = yaw;
    has_odom_ = true;
  }

  // Helper method to publish Twist velocity commands
  void move_robot(double linear_x, double angular_z) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    vel_pub_->publish(msg);
  }

  void stop_robot() { move_robot(0.0, 0.0); }

  // Goal Acceptance Callback
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "Goal Received: x=%.2f, y=%.2f, theta=%.2f rad",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Goal Cancellation Callback
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal cancellation requested.");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Execution Thread Setup
  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{std::bind(&GoToPose::execute, this, std::placeholders::_1),
                goal_handle}
        .detach();
  }

  // Action Control Loop Implementation
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing goal.");

    const auto goal = goal_handle->get_goal();
    
    // Store desired goal pose in class variable
    desired_pos_ = goal->goal_pos;

    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto result = std::make_shared<GoToPoseAction::Result>();

    rclcpp::Rate loop_rate(10); // 10 Hz control loop
    int feedback_counter = 0;   // Declare counter variable for 1 Hz feedback throttling

    // Tolerances specified in requirements
    const double pos_tolerance = 0.075;                        // 0.075 m (7.5 cm)
    const double orientation_tolerance = 10.0 * M_PI / 180.0; // 10 degrees in radians

    // RCLCPP_INFO(this->get_logger(), "Executing GoToPose control loop...");

    while (rclcpp::ok()) {
      // Check for cancel request
      if (goal_handle->is_canceling()) {
        stop_robot();
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled.");
        return;
      }

      // Ensure odometry data has been received
      if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for odometry message on /odom...");
        loop_rate.sleep();
        continue;
      }

      // Publish feedback ONCE PER SECOND (every 10 ticks of 10 Hz loop)
      feedback_counter++;
      if (feedback_counter >= 10) {
        feedback->current_pos = current_pos_;
        goal_handle->publish_feedback(feedback);
        feedback_counter = 0;
      }

      // Compute position error: distance = sqrt(dx^2 + dy^2)
      double dx = desired_pos_.x - current_pos_.x;
      double dy = desired_pos_.y - current_pos_.y;
      double pos_error = std::hypot(dx, dy);

      double linear_x = 0.0;
      double angular_z = 0.0;

      // Behavior Logic:
      // Case 1: Robot is far from target position (> 0.075 m)
      if (pos_error > pos_tolerance) {
        // Goal heading angle = atan2(dy, dx)
        double goal_direction = std::atan2(dy, dx);

        // Angular heading error normalized to [-pi, pi]
        double heading_error =
            normalize_angle(goal_direction - current_pos_.theta);

        // Turn towards goal direction
        angular_z = std::clamp(2.0 * heading_error, -1.0, 1.0);

        // Move forward at 0.2 m/s when heading error is small (< ~11.5 degrees / 0.2 rad)
        if (std::abs(heading_error) < 0.2) {
          linear_x = 0.2;
        } else {
          linear_x = 0.0;
        }
      }
      // Case 2: Target position reached (<= 0.075 m), rotate to final theta
      else {
        double yaw_error =
            normalize_angle(desired_pos_.theta - current_pos_.theta);

        // Rotate until final orientation is within 10 degrees tolerance
        if (std::abs(yaw_error) > orientation_tolerance) {
          linear_x = 0.0;
          angular_z = std::clamp(2.0 * yaw_error, -1.0, 1.0);
        }
        // Case 3: Goal position AND orientation reached within tolerance
        else {
          stop_robot();
          result->status = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(),
                      "Goal Completed: Reached target pose! Position error: %.3f m, Theta error: %.2f deg",
                      pos_error, std::abs(yaw_error) * 180.0 / M_PI);
          return;
        }
      }

      // Publish current velocity command
      move_robot(linear_x, angular_z);

      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}