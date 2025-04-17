#! /usr/bin/env python3
import modern_robotics as mr
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.wait_for_message import wait_for_message
import json
from incar.dataset.messages import TeleopCommandMessage, JointStateMessage

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from std_msgs.msg import String

import numpy as np
from tf_transformations import euler_matrix

ANGULAR_GAIN = 0.5
LINEAR_GAIN = 1.0

class Filter:
    def __init__(self, alpha: float, size):
        self.alpha: float = alpha
        self.size = size
        self.value: np.ndarray = np.zeros(size)

    def filter(self, new_value):
        self.value = (1-self.alpha)*self.value + self.alpha*np.array(new_value)
        return self.value.tolist()
    
    def reset(self):
        self.value = np.zeros(self.size)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.declare_parameter('starting_pose', [0.0107, 0.0169, 0.0276, -0.0077, 1.5693, 0.0123])
        self.declare_parameter('start_with_gripper_open', True)
        self._logger.info(f"Starting robot {self.get_namespace()}")
        if not self.get_namespace() in ['/left', '/right']:
            self._logger.error(f"Expected namespace to be either '/left' or '/right', but it was {self.get_namespace()}")
        self.teleop_controller = self.get_namespace()[1:]
        self.is_engaged = False
        self.dt = 0.1 # TODO: From parameter
        self.filter = Filter(0.5, 6)


        self.bot = InterbotixManipulatorXS(
            robot_model='vx300s',
            robot_name= self.get_namespace()[1:],
            group_name='arm',
            gripper_name='gripper',
        )

        robot_startup()

        # Initialise robot
        self.context.on_shutdown(self.reset_trajectory_speed) #TODO: This doesn't work!!!
        self.bot.arm.set_joint_positions(self.get_parameter('starting_pose').value, moving_time=3, blocking=True)
        self.new_pose = mr.se3ToVec(self.bot.arm.get_ee_pose())
        if self.get_parameter('start_with_gripper_open'):
            self.bot.gripper.release()
            self.gripper_is_open = True
        else:
            self.bot.gripper.grasp()
            self.gripper_is_open = False

        # Subscribe to teleop_commands
        self._logger.info("Initialised robot, waiting for first command...")
        teleop_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )
        _, command_msg = wait_for_message(String, self, '/teleop_commands', qos_profile=teleop_qos)
        self.current_command = json.loads(command_msg.data)
        if type(self.current_command) is str:
            self.current_command = json.loads(self.current_command)
        
        self.velocity_command = [0, 0, 0, 0, 0, 0]
        self.create_subscription(String, '/teleop_commands', self.command_callback, teleop_qos)

        self._logger.info("Command received, starting control loop!")
        self.create_timer(self.dt, self.control_loop)
        
    def command_callback(self, msg):
        #TODO: Use TeleopCommandMessage class
        message = json.loads(msg.data)
        if type(message) is str:
            message = json.loads(message)
        if not message["header"]["messageType"].split('.')[-1] == "TeleopCommandMessage":
            return
        
        current_engaged = (message[f"{self.teleop_controller}Buttons"]["primaryButton"] 
            and message[f"{self.teleop_controller}Buttons"]["gripValue"] > 0.5)

        if self.is_engaged == False and current_engaged == True:
            self.current_pose = self.bot.arm.get_ee_pose()

        self.is_engaged = current_engaged
        
        self.command = [
            LINEAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["linear"]["x"]*self.dt,
            LINEAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["linear"]["y"]*self.dt,
            LINEAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["linear"]["z"]*self.dt,
            -ANGULAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["angular"]["x"]*self.dt,
            -ANGULAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["angular"]["y"]*self.dt,
            -ANGULAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["angular"]["z"]*self.dt,
        ]
        # self._logger.info(f"{self.command}")
        self.current_command = message
        if self.is_engaged:
            self.velocity_command = self.filter.filter(self.command)

        if message[f"{self.teleop_controller}Buttons"]["joystickValue"]["x"] > 0.5 and message[f"{self.teleop_controller}Buttons"]["secondaryButton"]:
            self.is_engaged = False
            self.bot.arm.set_joint_positions(self.get_parameter('starting_pose').value, moving_time=3, blocking=True)
            self.is_engaged = current_engaged

    def control_loop(self):
        if not self.is_engaged:
            return

        # Control the arm
        T_base_target = np.identity(4)
        T_base_target[:3, :3] = euler_matrix(self.command[3], self.command[4], self.command[5])[:3, :3] @ self.current_pose[:3, :3]
        T_base_target[:3, 3] = self.current_pose[:3, 3] + self.command[:3]

        _, succes = self.bot.arm.set_ee_pose_matrix(
            T_base_target,
            custom_guess=self.bot.arm.get_joint_positions(),
            moving_time=self.dt*2,
            blocking=False
        )

        # self._logger.info(f"{self.bot.arm.get_joint_positions()}")

        if succes:
            self.current_pose = T_base_target

        # Control the gripper
        if self.current_command[f"{self.teleop_controller}Buttons"]["triggerValue"] > 0.75 and self.gripper_is_open:
            self.bot.gripper.grasp()
            self.gripper_is_open = False
        elif self.current_command[f"{self.teleop_controller}Buttons"]["triggerValue"] < 0.25 and not self.gripper_is_open:
            self.bot.gripper.release()
            self.gripper_is_open = True

    def reset_trajectory_speed(self):
        self.bot.arm.set_trajectory_time(moving_time=2)
        robot_shutdown()
        self._logger.info("Shutdown teleop node, reset trajectory time")

if __name__ == '__main__':
    rclpy.init()
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()