import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('simulated_agent_l1_bringup'),
        'config',
        'params.yaml'
    )

    mqtt_publish_position_node = Node(
        package = "simulated_agent_l1",
        name = "mqtt_publish_node",
        executable = "mqtt_publish_node",
        parameters = [config]
    )

    position_node = Node(
        package = "simulated_agent_l1",
        name = "position_node",
        executable = "position_node",
        parameters = [config]
    )
    

    ld.add_action(mqtt_publish_position_node)
    ld.add_action(position_node)
    return ld