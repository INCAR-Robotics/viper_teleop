#! /usr/bin/env python3
import math
import time
import traceback
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.wait_for_message import wait_for_message
import json

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_startup
from std_msgs.msg import String, Float32MultiArray

import numpy as np
from tf_transformations import euler_matrix
from incar.messages import RobotCommandMessage
from routines import parse_routines


COMMAND_TIMEOUT = 0.2

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        
        # Check if this node should be started
        self.declare_parameter('enabled', True)
        start_node = self.get_parameter('enabled').get_parameter_value().bool_value
        if not start_node:
            return

        self.declare_parameter('dt', 0.02)
        self.declare_parameter('home_position', [0.0107, 0.0169, 0.0276, -0.0077, 1.5693, 0.0123])
        self.declare_parameter('start_with_gripper_open', True)
        self.declare_parameter('gripper_enabled', True)
        self.declare_parameter('routines', "[]")
        self.dt = self.get_parameter('dt').value
        self.gripper_enabled = self.get_parameter('gripper_enabled').get_parameter_value().bool_value

        routine_string = json.loads(self.get_parameter('routines').value)
        self.routine_list = parse_routines(routine_string)
        self._logger.info(f"Loaded Routines: {self.routine_list}")

        self._logger.info(f"Starting robot {self.get_namespace()}")
        if not self.get_namespace() in ['/left', '/right']:
            self._logger.error(f"Expected namespace to be either '/left' or '/right', but it was {self.get_namespace()}")
        self.teleop_controller = self.get_namespace()[1:]

        # Initialize state
        self.is_engaged = True
        self.last_command_received = time.time()
        self.buffered_routine = None
        self.is_running_routine = False

        # Initialize robot
        self.bot = InterbotixManipulatorXS(
            robot_model='vx300s',
            robot_name= self.get_namespace()[1:],
            group_name='arm',
            gripper_name='gripper',
            gripper_pressure=1.0
        )

        robot_startup()

        self.ee_state_publisher = self.create_publisher(Float32MultiArray, 'ee_state', 1)
        self.position_command_publisher = self.create_publisher(Float32MultiArray, 'position_command', 1)

        # Initialise topics
        self.bot.arm.set_joint_positions(self.get_parameter('home_position').value, moving_time=5, blocking=True)
        self.current_pose = self.bot.arm.get_ee_pose()
        if self.get_parameter('start_with_gripper_open').value:
            self.bot.gripper.release()
            self.gripper_is_open = True
            self.gripper_command = 0.0
        else:
            self.bot.gripper.grasp()
            self.gripper_is_open = False
            self.gripper_command = 1.0

        # Subscribe to teleop_commands
        self._logger.info("Initialised robot, waiting for first command...")
        teleop_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.publishing_loop()
        self.create_timer(self.dt, self.publishing_loop, MutuallyExclusiveCallbackGroup())

        _, command_msg = wait_for_message(String, self, '/robot_commands', qos_profile=teleop_qos)
        self.current_command = json.loads(command_msg.data)
        if type(self.current_command) is str:
            self.current_command = json.loads(self.current_command)


        self.create_subscription(String, '/robot_commands', self.command_callback, teleop_qos)

        self._logger.info("Command received, starting control loop!")
        self.create_timer(self.dt, self.control_loop, MutuallyExclusiveCallbackGroup())
        
    def command_callback(self, msg):
        try:
            command_msg = RobotCommandMessage.from_json(msg.data)
            # self._logger.info(f"{command_msg}")
            
            if not f"teleop_action_{self.teleop_controller}" in command_msg.commands.keys():
                self.command = [0, 0, 0, 0, 0, 0]
            else:
                self.command = [
                    command_msg.commands[f"teleop_action_{self.teleop_controller}"][0]*self.dt,
                    command_msg.commands[f"teleop_action_{self.teleop_controller}"][1]*self.dt,
                    command_msg.commands[f"teleop_action_{self.teleop_controller}"][2]*self.dt,
                    -command_msg.commands[f"teleop_action_{self.teleop_controller}"][3]*self.dt,
                    -command_msg.commands[f"teleop_action_{self.teleop_controller}"][4]*self.dt,
                    -command_msg.commands[f"teleop_action_{self.teleop_controller}"][5]*self.dt,
                ]
                self.gripper_command = command_msg.commands[f"teleop_action_{self.teleop_controller}"][6]

            if command_msg.routines[self.teleop_controller] != -1:
                self.buffered_routine = self.routine_list[command_msg.routines[self.teleop_controller]]

            self.last_command_received = time.time()
        except Exception:
            self._logger.info(traceback.print_exc())

    def control_loop(self):
        try:
            if self.is_running_routine:
                self._logger.info("Skipping control loop due to routine")
                return
            
            if self.buffered_routine is not None:
                self._logger.info("going to run buffered_routine")
                self.is_running_routine = True
                time.sleep(0.1)
                self.buffered_routine.execute(self.bot, self._logger, self.current_pose)
                self.current_pose = self.bot.arm.get_ee_pose()
                if self.bot.gripper.get_gripper_position() > 1.35:
                    self._logger.info("Gripper is open at the end of routine")
                    self.gripper_is_open = True
                else:
                    self._logger.info("Gripper is closed at the end of routine")
                    self.gripper_is_open = False
                self.buffered_routine = None
                self._logger.info("Ran buffered routine")
                self.is_running_routine = False
                return
            
            if not self.is_engaged or time.time() - self.last_command_received > COMMAND_TIMEOUT:
                return

            if max(self.command[:3]) > 0.25*self.dt or min(self.command[:3]) < -0.25*self.dt:
                self._logger.info("EXCEEDED LINEAR LIMITS")
                return
            if max(self.command[3:6]) > 2*self.dt or min(self.command[3:6]) < -2*self.dt:
                self._logger.info("EXCEEDED ANGULAR LIMITS")
                return
        
            if self.gripper_enabled and self.gripper_command > 0.75 and self.gripper_is_open:
                self._logger.info("GRASPING")
                self.bot.gripper.grasp(0)
                self.gripper_is_open = False
            elif self.gripper_enabled and self.gripper_command < 0.25 and not self.gripper_is_open:
                self._logger.info("RELEASING")
                self.bot.gripper.release(0)
                self.gripper_is_open = True

            T_base_target = np.identity(4)
            T_base_target[:3, :3] = euler_matrix(self.command[3], self.command[4], self.command[5])[:3, :3] @ self.current_pose[:3, :3]
            T_base_target[:3, 3] = self.current_pose[:3, 3] + self.command[:3]

            _, succes = self.bot.arm.set_ee_pose_matrix(
                T_base_target,
                custom_guess=self.bot.arm.get_joint_positions(),
                moving_time=self.dt*1.1,
                blocking=False
            )

            if succes:
                self.current_pose = T_base_target
        except Exception:
            self._logger.info(traceback.print_exc())

    def publishing_loop(self):
        current_robot_pose = self.bot.arm.get_ee_pose()
        state_msg = Float32MultiArray()
        pose = get_pose_from_transform(current_robot_pose)
        msg = [
            pose[0],
            pose[1],
            pose[2],
            pose[3],
            pose[4],
            pose[5],
            float(self.bot.gripper.get_gripper_position())
        ]
        state_msg.data = msg
        self.ee_state_publisher.publish(state_msg)

        position_command_msg = Float32MultiArray()
        pose = get_pose_from_transform(self.current_pose)
        msg = [
            pose[0],
            pose[1],
            pose[2],
            pose[3],
            pose[4],
            pose[5],
            float(self.gripper_command)
        ]
        position_command_msg.data = msg
        self.position_command_publisher.publish(position_command_msg)

def get_pose_from_transform(transform):
    """
    Extracts the 6D Cartesian pose (position and orientation) from a 4x4 transformation matrix.
    
    Args:
        transform (numpy.ndarray): A 4x4 transformation matrix.
        
    Returns:
        tuple: A tuple containing the position (x, y, z) and orientation (roll, pitch, yaw) in radians.
    """
    # Extract the position
    position = transform[:3, 3]
    
    # Extract the orientation
    # We use the following convention: roll (x-axis), pitch (y-axis), yaw (z-axis)
    roll = np.arctan2(transform[2, 1], transform[2, 2])
    pitch = np.arctan2(-transform[2, 0], np.sqrt(transform[2, 1]**2 + transform[2, 2]**2))
    yaw = np.arctan2(transform[1, 0], transform[0, 0])
    
    return [position[0], position[1], position[2], roll, pitch, yaw]

if __name__ == '__main__':
    rclpy.init()
    teleop_node = TeleopNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(teleop_node)
    try:
        executor.spin()
        print("spinning stopped")
    except KeyboardInterrupt:
        pass
    finally:
        print("Going to destroy node")
        teleop_node.destroy_node()
        rclpy.shutdown()
        exit(0)