#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "device_interface/msg/motor_goal.hpp"
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

class JointTrajectoryConverter : public rclcpp::Node
{
public:
    JointTrajectoryConverter() : Node("joint_trajectory_converter")
    {
        std::string input_topic = this->declare_parameter("input_topic", "joint_trajectory"); 
        
        joint_map_["l_Base001__to__l_Link006"] = "J1";
        joint_map_["l_Link006__to__l_Link007"] = "J2";
        joint_map_["l_Link007__to__l_Link008"] = "J3";
        joint_map_["l_Link008__to__l_Link009"] = "J4";
        joint_map_["l_Link009__to__l_Link010"] = "J5";
        joint_map_["l_Link010__to__l_Link011"] = "J6";
        joint_map_["l_Link011__to__l_Claw001"] = "J7";

        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            input_topic, 10, 
            std::bind(&JointTrajectoryConverter::topic_callback, this, std::placeholders::_1));

        motor_goal_pub_ = this->create_publisher<device_interface::msg::MotorGoal>("motor_goal", 10);

        RCLCPP_INFO(this->get_logger(), "JointTrajectoryConverter initialized. Listening on %s", input_topic.c_str());
    }

private:
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<device_interface::msg::MotorGoal>::SharedPtr motor_goal_pub_;
    std::map<std::string, std::string> joint_map_;

class JointTrajectoryConverter : public rclcpp::Node
{
// ... existing code ...
    void topic_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty()) return;

        device_interface::msg::MotorGoal goal_msg;
        
        // Take the first point for instantaneous control
        const auto& point = msg->points[0];

        for (size_t i = 0; i < msg->joint_names.size(); ++i) {
            std::string joint_name = msg->joint_names[i];
            
            if (joint_map_.find(joint_name) != joint_map_.end()) {
                std::string motor_id = joint_map_[joint_name];
                
                goal_msg.motor_id.push_back(motor_id);

                // --- Position Control ---
                // We primarily send position (angle) commands.
                if (point.positions.size() > i)
                    goal_msg.goal_pos.push_back(point.positions[i]);
                else 
                    // Fallback to 0.0 or maybe keep previous position? 0.0 for safety now.
                    goal_msg.goal_pos.push_back(0.0);

                // --- Velocity & Torque ---
                // IMPORTANT: Set velocity and torque to NaN to force Unitree/Damiao drivers
                // to interpret this as Position Control mode properly.
                // Sending 0.0 might cause them to enter Velocity mode (with 0 target) instead.
                
                goal_msg.goal_vel.push_back(std::numeric_limits<double>::quiet_NaN());
                goal_msg.goal_tor.push_back(std::numeric_limits<double>::quiet_NaN());
            }
        }

        if (!goal_msg.motor_id.empty()) {
            motor_goal_pub_->publish(goal_msg);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointTrajectoryConverter>());
    rclcpp::shutdown();
    return 0;
}