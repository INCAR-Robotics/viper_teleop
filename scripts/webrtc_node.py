#! /usr/bin/env python3
import asyncio
import json
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import JointState as JointStateROS

from incar.messages.robot_state_pb2 import *
from incar.messages.primitives_pb2 import *
from incar.messages.sensor_data_pb2 import SensorData
from incar.webrtc.webrtc_connection import WebRTCConnection
from incar.webrtc.custom_tracks import CV2VideoStreamTrack


ROBOT_COMMAND_CHANNEL = "robot_command"
ROBOT_STATE_CHANNEL = "robot_state"
CORE_PUBLISHING_DT = 0.01 # TODO: Make configurable

class WebRTCNode(Node):
    def __init__(self):
        super().__init__('webrtc')
        self.declare_parameter('port', 9999)
        self.declare_parameter('local', False)
        self.declare_parameter('stream_cameras', True)
        self.declare_parameter('cameras', "{}")

        self.left_ee_state = None
        self.right_ee_state = None
        self.left_joint_states: JointStateROS = None
        self.right_joint_states: JointStateROS = None

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

        self.create_subscription(Float32MultiArray, '/left/ee_state', self.log_left_ee, qos_profile=joint_state_qos)
        self.create_subscription(Float32MultiArray, '/right/ee_state', self.log_right_ee, qos_profile=joint_state_qos)
        self.create_subscription(JointStateROS, '/left/joint_states', self.log_left_joint_states, qos_profile=joint_state_qos)
        self.create_subscription(JointStateROS, '/right/joint_states', self.log_right_joint_states, qos_profile=joint_state_qos)

        self.rtc_initialised = False

    def handle_msg(self, channel, msg: str):
        if channel == ROBOT_COMMAND_CHANNEL:
            try:
                ros_msg = String()
                # self._logger.info(f"{msg}")
                # self._logger.info(f"{str(msg)}")
                ros_msg.data = str(msg)
                self.teleop_pub.publish(ros_msg)
            except Exception as e:
                print(e)

    async def publish_core_data_loop(self):
        while True:
            self.rtc_initialised = False
            while self.rtc.get_peer().connectionState != "connected":
                await asyncio.sleep(0.2)
            
            await asyncio.sleep(1)
            self.rtc_initialised = True

            while self.rtc.get_peer().connectionState == "connected":
                start = time.time()

                if self.rtc.data_channels[ROBOT_STATE_CHANNEL].bufferedAmount != 0 or \
                self.rtc.data_channels["left.efforts"].bufferedAmount != 0 or \
                self.rtc.data_channels["right.efforts"].bufferedAmount != 0:
                    self._logger.info("[WARNING] NOT SENDING ANY DATA TO CORE DUE TO FULL BUFFERS")
                    await asyncio.sleep(0.02 - (end - start))
                    continue
                
                serialized_msg = self.get_state_message().SerializeToString()
                self.rtc.send_channel(ROBOT_STATE_CHANNEL, serialized_msg)

                self.send_efforts()

                end = time.time()
                await asyncio.sleep(CORE_PUBLISHING_DT - (end - start))

    def send_efforts(self):
        # LEFT
        if self.left_joint_states is not None:
            left_efforts = self.left_joint_states.effort[:7]
            serialized_msg = SensorData(data = list(left_efforts)).SerializeToString()
            self.rtc.send_channel("left.efforts", serialized_msg)

        # RIGHT
        if self.right_joint_states is not None:
            right_efforts = self.right_joint_states.effort[:7]
            serialized_msg = SensorData(data = list(right_efforts)).SerializeToString()
            self.rtc.send_channel("right.efforts", serialized_msg)

    def get_state_message(self) -> RobotState:
        message = RobotState(robotType="dual viper")

        # LEFT
        left_module = RobotModuleState()
        left_gripper_module = RobotModuleState()

        if self.left_ee_state is not None:
            left_ee=CartesianState(
                velocity=Velocity(
                    linear=Vector3(
                        x=self.left_ee_state[0],
                        y=self.left_ee_state[1],
                        z=self.left_ee_state[2]
                    ),
                    angular=Vector3(
                        x=self.left_ee_state[3],
                        y=self.left_ee_state[4],
                        z=self.left_ee_state[5],
                    )
                )
            )
            left_module.ee.CopyFrom(left_ee)

        if self.left_joint_states is not None:
            left_joint = JointState(
                positions = list(self.left_joint_states.position)[:6],
                velocities = list(self.left_joint_states.velocity)[:6]
            )
            left_module.joints.CopyFrom(left_joint)

            left_gripper_joints=JointState(
                positions = [self.left_joint_states.position[6]],
                velocities = [self.left_joint_states.velocity[6]]
            )
            left_gripper_module.joints.CopyFrom(left_gripper_joints)

        message.moduleStates["left.gripper"].CopyFrom(left_gripper_module)
        message.moduleStates["left.arm"].CopyFrom(left_module)

        # RIGHT
        right_module = RobotModuleState()
        right_gripper_module = RobotModuleState()

        if self.right_ee_state is not None:
            right_ee=CartesianState(
                velocity=Velocity(
                    linear=Vector3(
                        x=self.right_ee_state[0],
                        y=self.right_ee_state[1],
                        z=self.right_ee_state[2]
                    ),
                    angular=Vector3(
                        x=self.right_ee_state[3],
                        y=self.right_ee_state[4],
                        z=self.right_ee_state[5],
                    )
                )
            )
            right_module.ee.CopyFrom(right_ee)

        if self.right_joint_states is not None:
            right_joint = JointState(
                positions = list(self.right_joint_states.position)[:6],
                velocities = list(self.right_joint_states.velocity)[:6]
            )
            right_module.joints.CopyFrom(right_joint)

            right_gripper_joints=JointState(
                positions = [self.right_joint_states.position[6]],
                velocities = [self.right_joint_states.velocity[6]]
            )
            right_gripper_module.joints.CopyFrom(right_gripper_joints)

        message.moduleStates["right.gripper"].CopyFrom(right_gripper_module)
        message.moduleStates["right.arm"].CopyFrom(right_module)

        return message

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
            .add_channel("left.efforts")
            .add_channel("right.efforts")
        )

        tasks = [
            self.spin(),
            self.publish_core_data_loop(),
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
            task.result()

    def log_left_ee(self, msg):
        self.left_ee_state = msg.data.tolist()[:6]

    def log_right_ee(self, msg):
        self.right_ee_state = msg.data.tolist()[:6]

    def log_left_joint_states(self, joint_state_msg: JointStateROS):
        self.left_joint_states = joint_state_msg

    def log_right_joint_states(self, joint_state_msg: JointStateROS):
        self.right_joint_states = joint_state_msg
    


if __name__ == "__main__":
    rclpy.init()
    webrtc_node = WebRTCNode()
    webrtc_node.start()
