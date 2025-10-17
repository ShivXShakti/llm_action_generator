from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory    

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('llm_action_generator'),
        'config',
        'scene_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='llm_action_generator',
            executable='e_llm_ros',
            name='llama_client',
            output='screen',
            parameters=[config_file]
        )
    ])