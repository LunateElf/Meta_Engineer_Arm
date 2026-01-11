from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    # ================== 核心适配修改 ==================
    # 1. 这里的 "meta_engineer_arm" 必须对应你 moveit_config/config/ 目录下的文件名
    #    例如：如果你的 SRDF 文件叫 config/meta_engineer_arm.srdf，那这里就填 "meta_engineer_arm"
    #    如果你的 SRDF 叫 config/my_robot.srdf，那这里就填 "my_robot"
    # 2. package_name 填你生成的配置包名 "moveit_config"
    engineer_arm_moveit_config = MoveItConfigsBuilder("Robot", package_name="engineer_arm_moveit_config") \
        .to_moveit_configs()
    # =================================================

    # 定义节点
    run_move_group_node = Node(
        package="rm_engineer_control",
        # 确保这里和 CMakeLists.txt 中 add_executable 的名字一致
        executable="pose_subscriber_rpy",  
        output="screen",
        parameters=[
            engineer_arm_moveit_config.robot_description,
            engineer_arm_moveit_config.robot_description_semantic,
            engineer_arm_moveit_config.robot_description_kinematics,
            # 加载 OMPL 规划器
            engineer_arm_moveit_config.planning_pipelines,
            # 如果你的控制器配置文件叫 ros2_controllers.yaml (Setup Assistant 默认生成这个名字)
            # trajectory_execution 会自动处理，或者你可以显式指定：
            # moveit_config.trajectory_execution, 
        ],
    )

    return LaunchDescription([
        run_move_group_node
    ])