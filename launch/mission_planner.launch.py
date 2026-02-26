from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('mission_planner'),
        'config',
        'missionPlanner-config.yaml'
    )

    return LaunchDescription([
        Node(
            package='mission_planner',
            executable='mission_planner_node',
            name='mission_planner',
            output='screen',
            parameters=[config]
        )
    ])
