"""manipulator_board.launch.py

机械臂板卡协议适配层启动文件。

建议启动顺序（板卡本地）：
  1. ros2 launch openarm_bimanual_moveit_config demo.launch.py
  2. ros2 launch openarm_skills skills.launch.py
  3. ros2 launch openarm_perception perception.launch.py   # 可选
  4. ros2 launch openarm_manipulator_board manipulator_board.launch.py

多机环境注意事项：
  - 保证上位机与板卡 ROS_DOMAIN_ID 一致
  - ROS_LOCALHOST_ONLY=0
  - 时间同步（NTP）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("openarm_manipulator_board")
    default_cfg = os.path.join(pkg_share, "config", "manipulator_board.yaml")

    # 允许通过命令行参数覆盖常用配置
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_cfg,
            description="Path to manipulator_board.yaml",
        ),
        DeclareLaunchArgument(
            "topic_ns",
            default_value=EnvironmentVariable(
                "MANIPULATOR_TOPIC_NS", default_value="/robot_arm"
            ),
            description="ROS topic namespace for manipulator protocol topics",
        ),
        DeclareLaunchArgument(
            "board_id",
            default_value="arm-controller-01",
            description="Board identifier written into every status message",
        ),
        DeclareLaunchArgument(
            "default_arm",
            default_value="right",
            description="Arm used when command does not specify arm field",
        ),
        DeclareLaunchArgument(
            "use_default_poses",
            default_value="false",
            description="Use YAML default grasp/place poses when params omits them",
        ),
        Node(
            package="openarm_manipulator_board",
            executable="manipulator_board_node",
            name="manipulator_board_node",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "topic_ns":          LaunchConfiguration("topic_ns"),
                    "board_id":          LaunchConfiguration("board_id"),
                    "default_arm":       LaunchConfiguration("default_arm"),
                    "use_default_poses": LaunchConfiguration("use_default_poses"),
                },
            ],
        ),
    ])
