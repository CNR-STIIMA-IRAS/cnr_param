import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    ld=LaunchDescription()

    config = os.path.join(
        get_package_share_directory('cnr_param'),
        'config',
        'ros2parameters.yaml'
        )

    generic_node_with_parameters = Node(
        package='cnr_param',
        name='test_ros2_parameters_node',
        executable='test_ros2_parameters_node',
        parameters=[config],
        output='screen',
        emulate_tty=True
    )

    cnr_param_getter_node = Node(
        package='cnr_param',
        name='test_ros2_parameters_remote',
        executable='test_ros2_parameters_remote',
        output='screen',
        emulate_tty=True
    )

    ld.add_action(generic_node_with_parameters)
    ld.add_action(cnr_param_getter_node)
    return ld
