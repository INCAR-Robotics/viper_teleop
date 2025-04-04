#! /usr/bin/env python3
import modern_robotics as mr
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.wait_for_message import wait_for_message
import json

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from std_msgs.msg import String

import numpy as np


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
        self.bot.arm.go_to_home_pose(moving_time=3, blocking=True)
        # self.bot.arm.set_single_joint_position("wrist_angle", 1.57, moving_time=3, blocking=True)
        self.bot.arm.set_ee_pose_components(x=0.3,z=0.4, moving_time=3)
        self.new_pose = mr.se3ToVec(self.bot.arm.get_ee_pose())
        self.bot.gripper.release()
        self.gripper_is_open = True

        # Subscribe to teleop_commands
        self._logger.info("Initialised robot, waiting for first command...")
        teleop_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )
        _, command_msg = wait_for_message(String, self, '/teleop_commands', qos_profile=teleop_qos)
        self.current_command = json.loads(command_msg.data)
        self.velocity_command = [0, 0, 0, 0, 0, 0]
        self.create_subscription(String, '/teleop_commands', self.command_callback1, teleop_qos)

        self._logger.info("Command received, starting control loop!")
        self.create_timer(self.dt, self.control_loop)
        
    def command_callback1(self, msg):
        message = json.loads(msg.data)
        if not message["header"]["messageType"].split('.')[-1] == "TeleopCommandMessage":
            return
        
        current_engaged = (message[f"{self.teleop_controller}Buttons"]["primaryButton"] 
            and message[f"{self.teleop_controller}Buttons"]["gripValue"] > 0.5)
        
        if self.is_engaged == False and current_engaged == True:
            # NOTE: For some reason, modern robotics sets the pose vector as [yaw, pitch, roll, x, y, z]
            self.new_pose = mr.se3ToVec(self.bot.arm.get_ee_pose())

        self.is_engaged = current_engaged
        
        command = [
            ANGULAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["angular"]["x"],
            -ANGULAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["angular"]["y"],
            -ANGULAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["angular"]["z"],
            LINEAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["linear"]["x"],
            LINEAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["linear"]["y"],
            LINEAR_GAIN*self.current_command[f"{self.teleop_controller}Vel"]["linear"]["z"],
        ]
        self.current_command = message
        if self.is_engaged:
            self.velocity_command = self.filter.filter(command)

    def control_loop(self):
        if not self.is_engaged:
            return

        # Control the arm
        target_pose = np.array(self.new_pose) + np.array(self.velocity_command)*self.dt*2

        _, succes = self.bot.arm.set_ee_pose_components(
            x = target_pose[3],
            y = target_pose[4],
            z = target_pose[5],
            roll = target_pose[0],
            pitch = target_pose[1],
            yaw = target_pose[2],
            moving_time=self.dt*2,
            blocking=False
        )

        if succes:
            self.new_pose = (np.array(self.new_pose) + np.array(self.velocity_command)*self.dt).tolist()

        # command = self.current_command[f"{self.teleop_controller}Vel"]
        # self._logger.info(f"angular vel: {command['angular']}")

        # self.new_pose = [
        #     self.new_pose[0] + ANGULAR_GAIN*command["angular"]["x"]*self.dt*2,
        #     self.new_pose[1] + ANGULAR_GAIN*command["angular"]["y"]*self.dt*2,
        #     self.new_pose[2] + ANGULAR_GAIN*command["angular"]["z"]*self.dt*2,
        #     self.new_pose[3] + LINEAR_GAIN*command["linear"]["x"]*self.dt*2,
        #     self.new_pose[4] + LINEAR_GAIN*command["linear"]["y"]*self.dt*2,
        #     self.new_pose[5] + LINEAR_GAIN*command["linear"]["z"]*self.dt*2,
        # ]

        # _, succes = self.bot.arm.set_ee_pose_components(
        #     x = self.new_pose[3],
        #     y = self.new_pose[4],
        #     z = self.new_pose[5],
        #     roll = self.new_pose[0],
        #     pitch = self.new_pose[1],
        #     yaw = self.new_pose[2],
        #     moving_time=self.dt*2,
        #     blocking=False
        # )

        # if succes:
        #     self.new_pose = [
        #         self.new_pose[0] - ANGULAR_GAIN*command["angular"]["x"]*self.dt,
        #         self.new_pose[1] - ANGULAR_GAIN*command["angular"]["y"]*self.dt,
        #         self.new_pose[2] - ANGULAR_GAIN*command["angular"]["z"]*self.dt,
        #         self.new_pose[3] - LINEAR_GAIN*command["linear"]["x"]*self.dt,
        #         self.new_pose[4] - LINEAR_GAIN*command["linear"]["y"]*self.dt,
        #         self.new_pose[5] - LINEAR_GAIN*command["linear"]["z"]*self.dt,
        #     ]
        # else:
        #     self.new_pose = [
        #         self.new_pose[0] - ANGULAR_GAIN*command["angular"]["x"]*self.dt*2,
        #         self.new_pose[1] - ANGULAR_GAIN*command["angular"]["y"]*self.dt*2,
        #         self.new_pose[2] - ANGULAR_GAIN*command["angular"]["z"]*self.dt*2,
        #         self.new_pose[3] - LINEAR_GAIN*command["linear"]["x"]*self.dt*2,
        #         self.new_pose[4] - LINEAR_GAIN*command["linear"]["y"]*self.dt*2,
        #         self.new_pose[5] - LINEAR_GAIN*command["linear"]["z"]*self.dt*2,
        #     ]

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