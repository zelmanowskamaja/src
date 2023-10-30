from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('navigation'))

    navigation_cmd = Node(
            package='navigation',
            executable='navigation_node',
            name='navigation_node',
            parameters=[{'use_sim_time': True}, params_file],
            output='screen')
    
    
    ld = LaunchDescription(
        Node(
            package='navigation',
            namespace='nav1',
            executable='publisher.py',
            name='pub'
        )
    )    

    ld.add_action(navigation_cmd)

    return ld