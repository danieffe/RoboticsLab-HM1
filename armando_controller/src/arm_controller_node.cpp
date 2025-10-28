#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// ------------------- JointState Subscriber -------------------
class JointStateSubscriber : public rclcpp::Node
{
public:
    JointStateSubscriber()
    : Node("joint_state_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&JointStateSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState & msg) const {
        if (msg.position.size() >= 4) {
            RCLCPP_INFO(this->get_logger(), "-----Joint Positions-----");
            for (int i = 0; i < 4; i++) {
                RCLCPP_INFO(this->get_logger(), "Joint %d position: '%6f'", i, (double)msg.position[i]);
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

// ------------------- Position Controller Publisher -------------------
class PositionControllerPublisher : public rclcpp::Node
{
public:
    PositionControllerPublisher()
    : Node("position_controller_publisher"), count_(0)
    {
        position_commands_ = {
            {0.5, 0.0, 0.5, -0.5},  
            {1.0, 0.0, 1.0, -1.0},  
            {0.5, 0.0, 0.5, -0.5},  
            {0.0, 0.0, 0.0, 0.0}   
        };
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
        timer_ = this->create_wall_timer(5000ms, std::bind(&PositionControllerPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if (position_commands_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No position commands defined!");
            return;
        }

        auto current_command = position_commands_[count_];
        std_msgs::msg::Float64MultiArray commands_msg;
        commands_msg.data = current_command;

        RCLCPP_INFO(this->get_logger(), "Publishing position command set %zu", count_);
        for (size_t i = 0; i < current_command.size(); i++)
            RCLCPP_INFO(this->get_logger(), "  Joint %zu command: %6f", i, current_command[i]);

        publisher_->publish(commands_msg);
        count_ = (count_ + 1) % position_commands_.size();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
    std::vector<std::vector<double>> position_commands_;
};

// ------------------- Trajectory Controller Publisher -------------------
class TrajectoryControllerPublisher : public rclcpp::Node
{
public:
    TrajectoryControllerPublisher() 
    : Node("trajectory_controller_publisher"), count_(0) 
    {
        position_commands_ = {
            {0.5, 0.0, 0.5, -0.5},  
            {1.0, 0.0, 1.0, -1.0},  
            {0.5, 0.0, 0.5, -0.5},  
            {0.0, 0.0, 0.0, 0.0}   
        };

        joint_names_ = {"j0", "j1", "j2", "j3"};
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(5000ms, std::bind(&TrajectoryControllerPublisher::timer_callback, this));
    }

private:
    void timer_callback() 
    {
      auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
      trajectory_msg.header.stamp = this->get_clock()->now();
      trajectory_msg.joint_names = joint_names_;

      std::vector<double> current_command = position_commands_[count_];

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = current_command;

      size_t num_joints = joint_names_.size();
      point.velocities.resize(num_joints, 0.0);

      point.time_from_start = builtin_interfaces::msg::Duration();
      point.time_from_start.sec = 5;
      point.time_from_start.nanosec = 0;

      trajectory_msg.points.push_back(point);
      RCLCPP_INFO(this->get_logger(), "Trajectory command set %zu", count_);
      publisher_->publish(trajectory_msg);
      count_ = (count_ + 1) % position_commands_.size();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    size_t count_;
    std::vector<std::string> joint_names_;
    std::vector<std::vector<double>> position_commands_;
};

// ------------------- Main -------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto param_node = std::make_shared<rclcpp::Node>("controller_selector");
    std::string controller_type = param_node->declare_parameter<std::string>("controller_type", "position");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(param_node);

    auto joint_state_subscriber = std::make_shared<JointStateSubscriber>();
    executor.add_node(joint_state_subscriber);

    std::shared_ptr<PositionControllerPublisher> position_controller_publisher;
    std::shared_ptr<TrajectoryControllerPublisher> trajectory_controller_publisher;

    if (controller_type == "position") {
        RCLCPP_INFO(param_node->get_logger(), "Mode 'position' selected.");
        position_controller_publisher = std::make_shared<PositionControllerPublisher>();
        executor.add_node(position_controller_publisher);
    } else if (controller_type == "trajectory") {
        RCLCPP_INFO(param_node->get_logger(), "Mode 'trajectory' selected.");
        trajectory_controller_publisher = std::make_shared<TrajectoryControllerPublisher>();
        executor.add_node(trajectory_controller_publisher);
    }

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
