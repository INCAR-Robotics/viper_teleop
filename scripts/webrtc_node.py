#! /usr/bin/env python3
import socket
import sys
import time

import cv2
import numpy as np
from incar.webrtc.webrtc_connection import WebRTCConnection
from incar.webrtc.custom_tracks import CV2VideoStreamTrack, ZerosStreamTrack, VideoStreamTrack

import asyncio
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import JointState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from datetime import datetime


ROBOT_COMMAND_CHANNEL = "robot_command"
ROBOT_STATE_CHANNEL = "robot_state"

class WebRTCNode(Node):
    def __init__(self):
        super().__init__('webrtc')
        self.declare_parameter('port', 9999)
        self.declare_parameter('local', False)
        self.declare_parameter('stream_cameras', True)
        self.declare_parameter('cameras', "{}")

        
        self.left_state_msg = None
        self.right_state_msg = None
        self.position_command_msg = None

        teleop_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )
        self.teleop_pub = self.create_publisher(String, '/robot_commands', qos_profile=teleop_qos)

        joint_state_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # TODO: Get transforms from TF for base coords of robot state
        # TODO: Add EE frame in Robot state message?
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        # try:
        #     t = self.tf_buffer.lookup_transform('left', 'world', 0)
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform {'world'} to {'left'}: {ex}')

        self.create_subscription(Float32MultiArray, '/left/ee_state', self.set_left_message, qos_profile=joint_state_qos)
        self.create_subscription(Float32MultiArray, '/left/position_command', self.set_position_command_message, qos_profile=joint_state_qos)
        # self.create_subscription(JointState, '/right/joint_states', self.set_right_message, qos_profile=joint_state_qos)

    def handle_msg(self, channel, msg):
        # self._logger.info(f"received message on channel: {channel}")
        if channel == ROBOT_COMMAND_CHANNEL:
            try:
                ros_msg = String()
                ros_msg.data = msg
                self.teleop_pub.publish(ros_msg)
            except Exception as e:
                print(e)

    async def send_robot_state(self):
        while self.rtc.get_peer().connectionState != "connected":
            await asyncio.sleep(0.2)
        
        while True:
            start = time.time()

            if self.rtc.data_channels[ROBOT_STATE_CHANNEL].bufferedAmount != 0:
                self._logger.info("[WARNING] NOT SENDING ROBOT STATE DUE TO FULL BUFFER")
                await asyncio.sleep(0.02 - (end - start))
                continue

            if self.rtc.data_channels["position_command_plus_gripper"].bufferedAmount != 0:
                self._logger.info("[WARNING] NOT SENDING ROBOT STATE DUE TO FULL BUFFER")
                await asyncio.sleep(0.02 - (end - start))
                continue

            if self.left_state_msg is not None:
                self.rtc.send_channel(ROBOT_STATE_CHANNEL, self.left_state_msg)

            if self.position_command_msg is not None:
                self.rtc.send_channel("position_command_plus_gripper", self.position_command_msg)

            if self.right_state_msg is not None:
                self.rtc.send_channel(ROBOT_STATE_CHANNEL, self.right_state_msg)
            end = time.time()
            await asyncio.sleep(0.01 - (end - start))

    async def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            await asyncio.sleep(1e-4)

    def start(self):
        if self.get_parameter('local').get_parameter_value().bool_value:
            ip = "127.0.0.1"
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()

        self.rtc = (WebRTCConnection()
            .add_channel(ROBOT_COMMAND_CHANNEL, lambda msg: self.handle_msg(ROBOT_COMMAND_CHANNEL, msg))
            .add_channel(ROBOT_STATE_CHANNEL)
            .add_channel("position_command_plus_gripper")
        )

        tasks = [
            self.spin(),
            self.send_robot_state(),
            self.rtc.start_connection(ip, self.get_parameter('port').value, True),
        ]

        camera_dict = json.loads(self.get_parameter('cameras').value)
        if self.get_parameter('stream_cameras').get_parameter_value().bool_value:
            self._logger.info(f"Opening Cameras: {camera_dict}")
            for feature_name, camera_id in camera_dict.items():
                track = CV2VideoStreamTrack(camera_id)
                self.rtc.add_track(track, feature_name)
                tasks.append(track.update())

        future = asyncio.wait(
            tasks,
            return_when=asyncio.FIRST_EXCEPTION
        )

        self._logger.info("initialisation done")

        done, _pending = asyncio.get_event_loop().run_until_complete(future)
        for task in done:
            task.result()  # raises exceptions if any

    def set_position_command_message(self, msg):
        message = dict()
        message["data"] = msg.data.tolist()
        self.position_command_msg = message

    def set_left_message(self, msg): 
        state_message = dict()
        
        state_message["header"] = dict()
        state_message["header"]["timestamp"] = datetime.now().isoformat()
        state_message["header"]["messageType"] = "RobotStateMessage"
        state_message["header"]["version"] = "0.0.1"
        
        state_message["robotID"] = "left"

        state_message["baseFrame"] = dict()
        state_message["baseFrame"]["position"] = dict()
        state_message["baseFrame"]["position"]["x"] = 0
        state_message["baseFrame"]["position"]["y"] = 0
        state_message["baseFrame"]["position"]["z"] = 1
        state_message["baseFrame"]["rotation"] = dict()
        state_message["baseFrame"]["rotation"]["x"] = 0
        state_message["baseFrame"]["rotation"]["y"] = 0
        state_message["baseFrame"]["rotation"]["z"] = 0
        state_message["baseFrame"]["rotation"]["w"] = 1

        state_message["jointStates"] = msg.data.tolist()
        state_message["robotType"] = "dual_viper"

        self.left_state_msg = state_message

    def set_right_message(self, msg):
        state_message = dict()
        
        state_message["header"] = dict()
        state_message["header"]["timestamp"] = datetime.now().isoformat()
        state_message["header"]["messageType"] = "RobotStateMessage"
        state_message["header"]["version"] = "0.0.1"
        
        state_message["robotID"] = "right"

        state_message["baseFrame"] = dict()
        state_message["baseFrame"]["position"] = dict()
        state_message["baseFrame"]["position"]["x"] = 0
        state_message["baseFrame"]["position"]["y"] = 0.39
        state_message["baseFrame"]["position"]["z"] = 1
        state_message["baseFrame"]["rotation"] = dict()
        state_message["baseFrame"]["rotation"]["x"] = 0
        state_message["baseFrame"]["rotation"]["y"] = 0
        state_message["baseFrame"]["rotation"]["z"] = 1
        state_message["baseFrame"]["rotation"]["w"] = 0

        state_message["jointStates"] = msg.data.tolist()
        state_message["robotType"] = "dual_viper"

        self.right_state_msg = state_message


if __name__ == "__main__":
    rclpy.init()
    webrtc_node = WebRTCNode()
    webrtc_node.start()
