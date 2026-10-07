#! /usr/bin/env python3
import ast
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
from std_msgs.msg import String, Float32MultiArray, Int16

import numpy as np
from tf_transformations import euler_matrix
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
        self.declare_parameter('gripper_open_position', -1.0)
        self.declare_parameter('gripper_closed_position', -1.0)
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

        # Initialise topics
        self.bot.arm.set_joint_positions(self.get_parameter('home_position').value, moving_time=5, blocking=True)
        self.current_pose = self.bot.arm.get_ee_pose()
        self.cached_joints = self.bot.arm.get_joint_positions()

        # Calibrate the gripper's physical open/closed extremes (in raw motor radians, the
        # same units as get_gripper_position()) while in the default 'pwm' operating
        # mode, then switch to 'current_based_position' so gripper_command can drive it
        # continuously to any aperture in between, with torque still capped by gripper_pressure
        # instead of ramming a possibly-blocked target at full force like plain 'position' mode.
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_closed_position = self.get_parameter('gripper_closed_position').value

        if self.gripper_open_position < 0 or self.gripper_closed_position < 0:
            self.bot.gripper.core.robot_set_operating_modes('single', 'gripper', 'pwm')
            self.bot.gripper.release(1.0)
            self.gripper_open_position = self.bot.gripper.get_gripper_position()
            self._logger.info(f"Calibrated open position: {self.gripper_open_position}")
            self.bot.gripper.grasp(1.0)
            self.gripper_closed_position = self.bot.gripper.get_gripper_position()
            self._logger.info(f"Calibrated closed position: {self.gripper_closed_position}")

        self.bot.gripper.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
        # Current_Limit is an EEPROM register; torque must be off to write it.
        self.bot.gripper.core.robot_torque_enable('single', 'gripper', False)
        self.bot.gripper.core.robot_set_motor_registers(
            'single', 'gripper', 'Current_Limit', int(self.bot.gripper.gripper_value)
        )
        self.bot.gripper.core.robot_torque_enable('single', 'gripper', True)

        # Exposed on bot.gripper so routines.py can command full open/close via
        # robot_write_joint_command instead of grasp()/release(), which assumed pwm-mode
        # effort semantics and would be misread as huge position targets now.
        self.bot.gripper.open_position = self.gripper_open_position
        self.bot.gripper.closed_position = self.gripper_closed_position

        if self.get_parameter('start_with_gripper_open').value:
            self.gripper_is_open = True
            self.gripper_command = 0.0
            self.bot.gripper.core.robot_write_joint_command('gripper', self.gripper_open_position)
        else:
            self.gripper_is_open = False
            self.gripper_command = 1.0
            self.bot.gripper.core.robot_write_joint_command('gripper', self.gripper_closed_position)
        time.sleep(1.0)

        # Subscribe to teleop_commands
        self._logger.info("Initialised robot, waiting for first command...")
        teleop_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.publishing_loop()
        self.create_timer(self.dt, self.publishing_loop, MutuallyExclusiveCallbackGroup())

        _, command_msg = wait_for_message(Float32MultiArray, self, 'arm_commands', qos_profile=teleop_qos)
        self.arm_command_callback(command_msg)

        self.create_subscription(Float32MultiArray, 'arm_commands', self.arm_command_callback, teleop_qos)
        self.create_subscription(Float32MultiArray, 'gripper_commands', self.gripper_command_callback, teleop_qos)
        self.create_subscription(Float32MultiArray, 'routines', self.routine_callback, teleop_qos)

        self._logger.info("Command received, starting control loop!")
        self.create_timer(self.dt, self.control_loop, MutuallyExclusiveCallbackGroup())
    
    def arm_command_callback(self, msg: Float32MultiArray):
        self.command = [
            msg.data[0]*self.dt,
            msg.data[1]*self.dt,
            msg.data[2]*self.dt,
            -msg.data[3]*self.dt,
            -msg.data[4]*self.dt,
            -msg.data[5]*self.dt
        ]
        self.last_command_received = time.time()

    def gripper_command_callback(self, msg: Float32MultiArray):
        self.gripper_command = msg.data[0]

    def routine_callback(self, msg: Float32MultiArray):
        try:
            if self.is_running_routine: return

            routine_index = self.get_routine_index_from_array(msg.data)
            if routine_index is None: return

            if routine_index < len(self.routine_list):
                self.buffered_routine = self.routine_list[routine_index]
                self._logger.info(f"Buffered routine {self.buffered_routine}")
            else:
                self._logger.error(f"Received routine index {routine_index} but only have {len(self.routine_list)} routines loaded")
        except Exception:
            self._logger.info(traceback.print_exc())

    def get_routine_index_from_array(self, array) -> int | None:
        for i, value in enumerate(array):
            if value > 0.5: return i
        return None

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
                gripper_position = self.bot.gripper.get_gripper_position()
                if abs(gripper_position - self.gripper_open_position) < abs(gripper_position - self.gripper_closed_position):
                    self._logger.info("Gripper is open at the end of routine")
                    self.gripper_is_open = True
                else:
                    self._logger.info("Gripper is closed at the end of routine")
                    self.gripper_is_open = False
                time.sleep(0.1)
                # self.bot.arm.set_trajectory_time(self.dt*2)
                self.buffered_routine = None
                self._logger.info("Ran buffered routine")
                self.is_running_routine = False
                return
        
            if self.gripper_enabled:
                gripper_fraction = min(max(self.gripper_command, 0.0), 1.0)
                gripper_target = self.gripper_open_position + gripper_fraction * (
                    self.gripper_closed_position - self.gripper_open_position
                )
                self.bot.gripper.core.robot_write_joint_command('gripper', gripper_target)
                self.gripper_is_open = gripper_fraction < 0.5
            
            if max(max(self.command), -min(self.command)) == 0: return # Let joint_control_loop take over
            
            if not self.is_engaged or time.time() - self.last_command_received > COMMAND_TIMEOUT:
                return

            if max(self.command[:3]) > 0.25*self.dt or min(self.command[:3]) < -0.25*self.dt:
                self._logger.info("EXCEEDED LINEAR LIMITS")
                return
            if max(self.command[3:6]) > 2*self.dt or min(self.command[3:6]) < -2*self.dt:
                self._logger.info("EXCEEDED ANGULAR LIMITS")
                return

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
                self.cached_joints = self.bot.arm.get_joint_positions()
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