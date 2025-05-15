from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Create the launch description
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Launch the pick and place node
        Node(
            package='moveit2_scripts',
            executable='pick_and_place',
            name='pick_and_place',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        )
    ]) 