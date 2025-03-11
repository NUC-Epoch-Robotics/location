import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('chassis_serial_driver'), 'config', 'serial_driver.yaml')

    chassis_serial_driver_node = Node(
        package='chassis_serial_driver',
        executable='chassis_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
        remappings=[('cmd_vel', 'Odometry')]
    )

    return LaunchDescription([chassis_serial_driver_node])
