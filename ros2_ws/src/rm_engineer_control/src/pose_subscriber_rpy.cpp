#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/twist.hpp> 
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

// Degree to Radian conversion
#define DEG2RAD(x) ((x) * M_PI / 180.0)

using std::placeholders::_1;

class EngineerArmSubscriberRPY : public rclcpp::Node
{
public:
  EngineerArmSubscriberRPY() : Node("engineer_arm_subscriber_rpy")
  {
    // Initialization in init_moveit()
  }

  void init_moveit()
  {
    // ================== 核心修改点 ==================
    // 这里的名字必须和你 MoveIt Setup Assistant 设置的 "Planning Group" 名字完全一致！
    // 之前是 "ur_manipulator"，现在假设你是 "engineer_arm"
    // 如果你在 Setup Assistant 里起名叫 "arm" 或 "arm_group"，请务必在这里修改！
    static const std::string PLANNING_GROUP = "meta_engineer_arm"; 
    
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    
    // ===============================================

    // Velocity configuration
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);

    // Subscribe Twist messages
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/engineer/target_pose_rpy", 10, std::bind(&EngineerArmSubscriberRPY::topic_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "=== RPY Subscriber ready ===");
    RCLCPP_INFO(this->get_logger(), "Listening to /engineer/target_pose_rpy ...");
    RCLCPP_INFO(this->get_logger(), "Planning Group: %s", PLANNING_GROUP.c_str());
  }

  // ... (下方的 topic_callback 和 main 函数完全不用变，保持原样即可) ...
private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double target_x = msg->linear.x;
    double target_y = msg->linear.y;
    double target_z = msg->linear.z;

    double roll  = DEG2RAD(msg->angular.x);
    double pitch = DEG2RAD(msg->angular.y);
    double yaw   = DEG2RAD(msg->angular.z);

    RCLCPP_INFO(this->get_logger(), "Received command -> Pos:[%.2f, %.2f, %.2f]", target_x, target_y, target_z);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw); 
    q.normalize(); 

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_x;
    target_pose.position.y = target_y;
    target_pose.position.z = target_z;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    move_group_->setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Success! Executing...");
      move_group_->execute(my_plan);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Plan Failed! Check workspace limits or collision.");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EngineerArmSubscriberRPY>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  node->init_moveit();
  executor.spin();
  rclcpp::shutdown();
  return 0;
}