from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    left_teleop_node = Node(
        package = 'viper_teleop',
        namespace = 'left',
        executable = 'teleop_node.py',
        name = 'teleop',
        output = 'screen',
        parameters=[{
            'dt': 0.01,
            # 'starting_pose': [-0.8483, -0.2638, 0.4955, 1.6076, 1.6245, -0.2884],
            'starting_pose': [-0.0107, -0.2286, 0.2291, 0.0077, 1.3668, -0.0674],
            'start_with_gripper_open': False}]
    )

    right_teleop_node = Node(
        package = 'viper_teleop',
        namespace = 'right',
        executable = 'teleop_node.py',
        name = 'teleop',
        output = 'screen',
        parameters=[{'starting_pose': [-0.0107, -0.2286, 0.2291, 0.0077, 1.3668, -0.0674],
                     'start_with_gripper_open': False}]
    )

    webrtc_node = Node(
        package = 'viper_teleop',
        executable = 'webrtc_node.py',
        name = 'webrtc',
        output = 'screen',
        emulate_tty = True
    )

    return LaunchDescription([
        webrtc_node,
        left_teleop_node,
        right_teleop_node
    ])