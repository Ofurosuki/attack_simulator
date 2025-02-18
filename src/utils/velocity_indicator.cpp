#include "rclcpp/rclcpp.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "std_msgs/msg/float64.hpp" // For publishing velocity as a Float64 message
#include <cmath>

class VelocityCalculationNode : public rclcpp::Node {
public:
    VelocityCalculationNode() : Node("velocity_calculation_node") {
        // Subscriber to /vehicle/status/velocity_status
        velocity_subscription_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status",
            10,
            std::bind(&VelocityCalculationNode::velocityCallback, this, std::placeholders::_1)
        );

        // Publisher to /calculated_velocity
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/calculated_velocity", 10);
    }

private:
    void velocityCallback(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
        // Extract latitude and longitude velocities
        double lat_velocity = msg->lateral_velocity;
        double lon_velocity = msg->longitudinal_velocity;

        // Calculate the magnitude of the velocity vector
        double velocity_magnitude = std::sqrt(lat_velocity * lat_velocity + lon_velocity * lon_velocity);

        // Log the calculated velocity
        RCLCPP_INFO(this->get_logger(), "Calculated velocity: %.2f m/s", velocity_magnitude);

        // Publish the calculated velocity
        auto velocity_msg = std_msgs::msg::Float64();
        velocity_msg.data = velocity_magnitude*3.6;
        velocity_publisher_->publish(velocity_msg);
    }

    rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityCalculationNode>());
    rclcpp::shutdown();
    return 0;
}
