from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()
    
    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", moveit_config.robot_description.rviz_config],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(
        [rviz_node]
    )