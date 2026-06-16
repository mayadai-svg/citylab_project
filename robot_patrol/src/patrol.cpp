#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <thread>

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
        // - This angle represents the direction of the most open area, and therefore the
        // direction the robot should rotate toward.
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