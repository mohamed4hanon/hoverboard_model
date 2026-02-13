import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to your files
    # Make sure these paths match where your files actually are!
    urdf_path = os.path.join(os.getcwd(), 'hoverboard_model.urdf')
    rviz_config_path = os.path.join(os.getcwd(), 'hoverboard.rviz')
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 2. RViz2 with PRESET SETTINGS
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] # This loads your saved settings!
        )
    ])
