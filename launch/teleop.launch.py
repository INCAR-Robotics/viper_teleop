from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    left_teleop_node = Node(
        package = 'viper_teleop',
        namespace = 'left',
        executable = 'teleop_node.py',
        name = 'teleop',
        output = 'screen',
        parameters=[{'starting_pose': [0.1457, -0.6136, 0.8084, -1.3315, 0.9603, 1.5447]}]
    )

    right_teleop_node = Node(
        package = 'viper_teleop',
        namespace = 'right',
        executable = 'teleop_node.py',
        name = 'teleop',
        output = 'screen',
        parameters=[{'starting_pose': [-0.8483, -0.2638, 0.4955, 1.6076, 1.6245, -0.2884]}]
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