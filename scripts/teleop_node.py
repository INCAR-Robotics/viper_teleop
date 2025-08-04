#! /usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.wait_for_message import wait_for_message
import json

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_startup
from std_msgs.msg import String, Float32MultiArray

import numpy as np
from tf_transformations import euler_matrix
from incar.messages import RobotCommandMessage, GO_HOME_COMMAND

COMMAND_TIMEOUT = 0.2

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('starting_pose', [0.0107, 0.0169, 0.0276, -0.0077, 1.5693, 0.0123])
        self.declare_parameter('start_with_gripper_open', True)
        self._logger.info(f"Starting robot {self.get_namespace()}")
        if not self.get_namespace() in ['/left', '/right']:
            self._logger.error(f"Expected namespace to be either '/left' or '/right', but it was {self.get_namespace()}")
        self.teleop_controller = self.get_namespace()[1:]
        self.dt = self.get_parameter('dt').value
        self.is_engaged = True
        self.last_command_received = time.time()

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

        # Initialise robot
        self.bot.arm.set_joint_positions(self.get_parameter('starting_pose').value, moving_time=5, blocking=True)
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

        pose = get_pose_from_transform(self.current_pose)
        self.position_command_plus_gripper = [
            pose[0],
            pose[1],
            pose[2],
            pose[3],
            pose[4],
            pose[5],
            float(self.gripper_command)
        ]
        self._logger.info(f"dt is {self.dt}")
        self.publishing_loop()
        self.create_timer(self.dt, self.publishing_loop)

        _, command_msg = wait_for_message(String, self, '/robot_commands', qos_profile=teleop_qos)
        self.current_command = json.loads(command_msg.data)
        if type(self.current_command) is str:
            self.current_command = json.loads(self.current_command)

        self.create_subscription(String, '/robot_commands', self.command_callback, teleop_qos)

        self._logger.info("Command received, starting control loop!")
        self.create_timer(self.dt, self.control_loop)
        
    def command_callback(self, msg):  
        command_msg = RobotCommandMessage.from_json(msg.data)
        
        if not f"teleop_action_{self.teleop_controller}" in command_msg.commands.keys():
            self._logger.info("No teleop control command")
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
            # self._logger.info(f"{command_msg.commands[f"teleop_action_{self.teleop_controller}"]}")
            self.gripper_command = command_msg.commands[f"teleop_action_{self.teleop_controller}"][6]
            # self._logger.info(f"{self.gripper_command}")
        # if "position_command_plus_gripper" in command_msg.commands.keys():
        #     self.position_command_plus_gripper = command_msg.commands["position_command_plus_gripper"]

        if GO_HOME_COMMAND in command_msg.string_commands[self.teleop_controller]:
            self.is_engaged = False
            self.bot.arm.set_joint_positions(self.get_parameter('starting_pose').value, moving_time=3, blocking=True)
            self.current_pose = self.bot.arm.get_ee_pose()
            if self.get_parameter('start_with_gripper_open').value:
                self.bot.gripper.release()
                self.gripper_is_open = True
            else:
                self.bot.gripper.grasp()
                self.gripper_is_open = False
            self.is_engaged = True

        self.last_command_received = time.time()

    def control_loop(self):
        if not self.is_engaged or time.time() - self.last_command_received > COMMAND_TIMEOUT:
            return

        if max(self.command[:3]) > 0.25*self.dt or min(self.command[:3]) < -0.25*self.dt:
            self._logger.info("EXCEEDED LINEAR LIMITS")
            return
        if max(self.command[3:6]) > 2*self.dt or min(self.command[3:6]) < -2*self.dt:
            self._logger.info("EXCEEDED ANGULAR LIMITS")
            return
        

        # if max(self.command) == 0:
        #     self._logger.info("did not get any left/right velocity commands, going to position_command_plus_gripper")
        #     self.gripper_command = self.position_command_plus_gripper[-1]

    
        if self.gripper_command > 0.75 and self.gripper_is_open:
            self._logger.info("GRASPING")
            self.bot.gripper.grasp(0)
            self.gripper_is_open = False
        elif self.gripper_command < 0.25 and not self.gripper_is_open:
            self._logger.info("RELEASING")
            self.bot.gripper.release(0)
            self.gripper_is_open = True

        # # Control the arm
        # if max(self.command) == 0:
        #     old_pose = get_pose_from_transform(self.current_pose)
        #     if max(abs(np.array(old_pose[:3]) - np.array(self.position_command_plus_gripper[:3]))) > 0.4*self.dt:
        #         self._logger.info("EXCEEDED LINEAR LIMITS")
        #         return
        #     if max(abs(np.array(old_pose[3:]) - np.array(self.position_command_plus_gripper[3:6]))) > 2*self.dt:
        #         self._logger.info("EXCEEDED ANGULAR LIMITS")
        #         return
            
        #     T_base_target = np.identity(4)
        #     T_base_target[:3, :3] = euler_matrix(self.position_command_plus_gripper[3],
        #                                  self.position_command_plus_gripper[4],
        #                                  self.position_command_plus_gripper[5])[:3, :3]
        #     T_base_target[:3, 3] = self.position_command_plus_gripper[:3]
        # else:
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
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()