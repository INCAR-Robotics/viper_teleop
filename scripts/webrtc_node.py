#! /usr/bin/env python3
from incar.webrtc.webrtc_connection import WebRTCConnection
from incar.webrtc.custom_tracks import CV2VideoStreamTrack

import asyncio
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import JointState


TELEOP_COMMAND_CHANNEL = "teleop_command"
ROBOT_STATE_CHANNEL = "robot_state"

class WebRTCNode(Node):
    def __init__(self):
        super().__init__('webrtc')
        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('port', 9999)
        
        self.left_state_msg = None
        self.right_state_msg = None

        teleop_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )
        self.teleop_pub = self.create_publisher(String, '/teleop_commands', qos_profile=teleop_qos)

        joint_state_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )
        self.create_subscription(JointState, '/left/joint_states', self.set_left_message, qos_profile=joint_state_qos)
        self.create_subscription(JointState, '/right/joint_states', self.set_right_message, qos_profile=joint_state_qos)

    def handle_msg(self, channel, msg):
        # self._logger.info(f"received message on channel: {channel}")
        if channel == TELEOP_COMMAND_CHANNEL:
            try:
                ros_msg = String()
                ros_msg.data = msg
                self.teleop_pub.publish(ros_msg)
            except Exception as e:
                print(e)


    async def send_robot_state(self):
        while True:
            if self.rtc.get_peer().connectionState != "connected":
                await asyncio.sleep(0.02)
                continue

            await asyncio.sleep(0.02)
            continue
            if self.left_state_msg is not None:
                self.rtc.send_channel(ROBOT_STATE_CHANNEL, json.dumps(self.left_state_msg))
            if self.right_state_msg is not None:
                self.rtc.send_channel(ROBOT_STATE_CHANNEL, json.dumps(self.right_state_msg))

    async def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            await asyncio.sleep(1e-4)

    def start(self):
        # self.rtc = (WebRTCConnection()
        #     .add_on_datachannel_event(on_msg_event = lambda channel, msg: self.handle_msg(channel, msg))
        #     .add_track(CV2VideoStreamTrack(0)))
        
        self.rtc = (WebRTCConnection()
                    .add_channel(TELEOP_COMMAND_CHANNEL, lambda msg: self.handle_msg(TELEOP_COMMAND_CHANNEL, msg))
                    .add_channel(ROBOT_STATE_CHANNEL))

        future = asyncio.wait(
            [
                self.spin(),
                self.send_robot_state(),
                self.rtc.start_connection(self.get_parameter('ip').value, self.get_parameter('port').value, True)
            ], 
            return_when=asyncio.FIRST_EXCEPTION
        )

        self._logger.info("initialisation done")

        done, _pending = asyncio.get_event_loop().run_until_complete(future)
        for task in done:
            task.result()  # raises exceptions if any

    def set_left_message(self, msg): 
        state_message = dict()
        
        state_message["header"] = dict()
        state_message["header"]["timestamp"] = ""
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

        state_message["jointStates"] = msg.position
        state_message["robotType"] = "dual_viper"

        self.left_state_msg = state_message

    def set_right_message(self, msg):
        state_message = dict()
        
        state_message["header"] = dict()
        state_message["header"]["timestamp"] = ""
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

        state_message["jointStates"] = msg.position
        state_message["robotType"] = "dual_viper"

        self.right_state_msg = state_message


if __name__ == "__main__":
    rclpy.init()
    webrtc_node = WebRTCNode()
    webrtc_node.start()
