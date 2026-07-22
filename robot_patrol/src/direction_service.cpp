#include <rclcpp/rclcpp.hpp>
#include <services_quiz_srv/srv/turn.hpp>
#include <string>
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>

class DirectionService : public rclcpp::Node
{
public:
    DirectionService() : Node("direction_service")
    {
        // Create a service to handle turn requests
        std::string name_service = "/direction_service";
        service_ = this->create_service<robot_patrol::srv::GetDirection>(
            name_service,
            std::bind(&DirectionService::handle_direction_request, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // Create publisher to command robot velocity
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "%s Custom Interface Service Server Ready...", name_service.c_str());
    }

private:

    // Use the data received in the request (request) part of the message to make the robot spin:
    // - The robot will spin to one side or another depending on the direction value.
    // - Use the angular_velocity value to determine the velocity at which the robot will spin.
    // - Use the time value to determine the duration of the spin.
    // - Finally, it must return True if everything went okay in the success variable, which in this case is simply that the service server callback completes.

    void handle_direction_request(
        const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
        std::shared_ptr<robot_patrol::srv::GetDirection::Response> response)

    {
        RCLCPP_INFO(this->get_logger(), "Received Request -> Direction: %s, Velocity: %0.2f rad/s, Time: %0.2f s",
                    request->direction.c_str(), request->angular_velocity, request->time);

        geometry_msgs::msg::Twist twist_msg;

        // Determine the sign of the velocity based on direction
        // ROS convention: Positive Z = Left (CCW), Negative Z = Right (CW)
        double velocity = request->angular_velocity;
        if (request->direction == "right") {
            velocity = -velocity;
        }

        twist_msg.angular.z = velocity;

        // Loop control variables
        rclcpp::Rate loop_rate(10); // 10 Hz frequency
        auto start_time = this->get_clock()->now();
        auto duration = rclcpp::Duration::from_seconds(request->time);

        RCLCPP_INFO(this->get_logger(), "Turning the robot...");

        // Keep publishing the velocity until the time duration is reached
        while (rclcpp::ok() && (this->get_clock()->now() - start_time) < duration) {
            publisher_->publish(twist_msg);
            loop_rate.sleep();
        }

        // Stop the robot after the time has elapsed
        twist_msg.angular.z = 0.0;
        publisher_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Robot stopped.");

        // Set response success to true
        response->success = true;
    }

    rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}