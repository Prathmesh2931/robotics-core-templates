#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robotics-core-templates/kinematics/differential_drive.hpp"

using namespace robotics_core;

class DiffDriveNode : public rclcpp::Node {
public:
    DiffDriveNode() : Node("diff_drive_node") {
        this->declare_parameter<double>("track_width", 0.2);
        this->declare_parameter<double>("wheel_radius", 0.05);
        this->declare_parameter<double>("max_speed", 10.0);
        this->declare_parameter<std::vector<int>>("motor_directions", {1, 1});

        DifferentialParams params;
        params.track_width = this->get_parameter("track_width").as_double();
        params.wheel_radius = this->get_parameter("wheel_radius").as_double();
        params.max_speed = this->get_parameter("max_speed").as_double();

        // Convert motor_directions from long int from Ros2 params  to int for struct
        auto dirs = this->get_parameter("motor_directions").as_integer_array(); 
        for(auto d : dirs) params.motor_directions.push_back(static_cast<int>(d));

        kinematics_ = std::make_unique<DifferentialKinematics>(params); // Template Class for Differential Kinematics
        RCLCPP_INFO(this->get_logger(), "Differential Drive Node Initialized");

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DiffDriveNode::cmdVelCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speeds", 10);
    }
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto speeds = kinematics_->calculateWheelSpeeds(msg->linear.x, msg->angular.z);

        auto output_msg = std_msgs::msg::Float64MultiArray();
        output_msg.data = speeds;
        publisher_->publish(output_msg);
    }

    std::unique_ptr<DifferentialKinematics> kinematics_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveNode>());
    rclcpp::shutdown();
    return 0; 
}