from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    left_teleop_node = Node(
        package = 'viper_teleop',
        namespace = 'left',
        executable = 'teleop_node.py',
        name = 'teleop',
        output = 'screen'
    )

    right_teleop_node = Node(
        package = 'viper_teleop',
        namespace = 'right',
        executable = 'teleop_node.py',
        name = 'teleop',
        output = 'screen'
    )

    webrtc_node = Node(
        package = 'viper_teleop',
        executable = 'webrtc_node.py',
        name = 'webrtc',
        output = 'screen',
        emulate_tty = True,
        parameters=[{'ip': '192.168.200.130', 
                     'port': 9999}]
    )

    return LaunchDescription([
        webrtc_node,
        left_teleop_node,
        right_teleop_node
    ])