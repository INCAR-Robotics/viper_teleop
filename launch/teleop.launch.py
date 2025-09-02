from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task_config_arg = DeclareLaunchArgument(
        'task_config',
        default_value='pick_cube.yaml',
        description='Name of the configuration file for the task'
    )
    
    stream_cameras_arg = DeclareLaunchArgument(
        'stream_cameras',
        default_value='True',
        description='False if cameras are connected directly to the core. True otherwise'
    )

    connect_locally_arg = DeclareLaunchArgument(
        'local',
        default_value='False',
        description='True if the core is running on the same PC as the robots. False otherwise'
    )
    
    config = PathJoinSubstitution([
        FindPackageShare('viper_teleop'), 
        'config', 
        'tasks', 
        LaunchConfiguration('task_config')
    ])


    # left_teleop_node = Node(
    #     package = 'viper_teleop',
    #     namespace = 'left',
    #     executable = 'teleop_node.py',
    #     name = 'teleop',
    #     output = 'screen',
    #     parameters=[{
    #         'dt': 0.01,
    #         # 'starting_pose': [-0.8483, -0.2638, 0.4955, 1.6076, 1.6245, -0.2884],
    #         'starting_pose': [-0.0107, -0.2286, 0.2291, 0.0077, 1.3668, -0.0674],
    #         'start_with_gripper_open': False}]
    # )

    # right_teleop_node = Node(
    #     package = 'viper_teleop',
    #     namespace = 'right',
    #     executable = 'teleop_node.py',
    #     name = 'teleop',
    #     output = 'screen',
    #     parameters=[{'starting_pose': [-0.0107, -0.2286, 0.2291, 0.0077, 1.3668, -0.0674],
    #                  'start_with_gripper_open': False}]
    # )

    left_teleop_node = Node(
        package = 'viper_teleop',
        namespace = 'left',
        executable = 'teleop_node.py',
        name = 'teleop',
        output = 'screen',
        parameters = [config]
    )

    right_teleop_node = Node(
        package = 'viper_teleop',
        namespace = 'right',
        executable = 'teleop_node.py',
        name = 'teleop',
        output = 'screen',
        parameters = [config]
    )

    webrtc_node = Node(
        package = 'viper_teleop',
        executable = 'webrtc_node.py',
        name = 'webrtc',
        output = 'screen',
        emulate_tty = True,
        parameters = [
            config,
            {
                'local': LaunchConfiguration('local'),
                'stream_cameras': LaunchConfiguration('stream_cameras')
            }
        ]
    )

    return LaunchDescription([
        task_config_arg,
        stream_cameras_arg,
        connect_locally_arg,
        webrtc_node,
        left_teleop_node,
        right_teleop_node
    ])