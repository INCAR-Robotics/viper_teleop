#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from incar_networking.robot_interface import IncarRobotInterface


INCAR_DT = 0.01 # TODO: Make configurable

class WebRTCNode(Node):
    def __init__(self):
        super().__init__('webrtc')

        self.left_ee_state = None
        self.right_ee_state = None
        self.left_joint_states: JointState = None
        self.right_joint_states: JointState = None

        teleop_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )
        self.left_arm_commands = self.create_publisher(Float32MultiArray, '/left/arm_commands', qos_profile=teleop_qos)
        self.left_gripper_commands = self.create_publisher(Float32MultiArray, '/left/gripper_commands', qos_profile=teleop_qos)
        self.right_arm_commands = self.create_publisher(Float32MultiArray, '/right/arm_commands', qos_profile=teleop_qos)
        self.right_gripper_commands = self.create_publisher(Float32MultiArray, '/right/gripper_commands', qos_profile=teleop_qos)

        joint_state_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        self.create_subscription(Float32MultiArray, '/left/ee_state', self.log_left_ee, qos_profile=joint_state_qos)
        self.create_subscription(Float32MultiArray, '/right/ee_state', self.log_right_ee, qos_profile=joint_state_qos)
        self.create_subscription(JointState, '/left/joint_states', self.log_left_joint_states, qos_profile=joint_state_qos)
        self.create_subscription(JointState, '/right/joint_states', self.log_right_joint_states, qos_profile=joint_state_qos)

    def spin(self, interface: IncarRobotInterface):
        rclpy.spin_once(self, timeout_sec=0)

    def log_left_ee(self, msg):
        self.left_ee_state = msg.data.tolist()[:6]

    def log_right_ee(self, msg):
        self.right_ee_state = msg.data.tolist()[:6]

    def log_left_joint_states(self, joint_state_msg: JointState):
        self.left_joint_states = joint_state_msg

    def log_right_joint_states(self, joint_state_msg: JointState):
        self.right_joint_states = joint_state_msg
    
    def publish_state(self, interface: IncarRobotInterface):
        if self.left_ee_state is not None and self.left_joint_states is not None:
            interface.set_robot_state(
                "left.arm", 
                ee_pose=self.left_ee_state, 
                joint_pos=self.left_joint_states.position[:6],
                joint_vel=self.left_joint_states.velocity[:6],
                joint_effort=self.left_joint_states.effort[:6]
            )
            interface.set_robot_state(
                "left.gripper", 
                joint_pos=[self.left_joint_states.position[6]],
                joint_vel=[self.left_joint_states.velocity[6]],
                joint_effort=[self.left_joint_states.effort[6]]
            )
        if self.right_ee_state is not None and self.right_joint_states is not None:
            interface.set_robot_state(
                "right.arm", 
                ee_pose=self.right_ee_state, 
                joint_pos=self.right_joint_states.position[:6],
                joint_vel=self.right_joint_states.velocity[:6],
                joint_effort=self.right_joint_states.effort[:6]
            )
            interface.set_robot_state(
                "right.gripper", 
                joint_pos=[self.right_joint_states.position[6]],
                joint_vel=[self.right_joint_states.velocity[6]],
                joint_effort=[self.right_joint_states.effort[6]]
            )
        interface.publish_state()

if __name__ == "__main__":
    rclpy.init()
    webrtc_node = WebRTCNode()
    interface = IncarRobotInterface(
        INCAR_DT,
        command_hooks = {
            "right.commands.arm.ee.velocity": lambda x: webrtc_node.right_arm_commands.publish(Float32MultiArray(data=x)),
            "left.commands.arm.ee.velocity": lambda x: webrtc_node.left_arm_commands.publish(Float32MultiArray(data=x)),
            "right.commands.gripper.openclose": lambda x: webrtc_node.right_gripper_commands.publish(Float32MultiArray(data=x)),
            "left.commands.gripper.openclose": lambda x: webrtc_node.left_gripper_commands.publish(Float32MultiArray(data=x))
        },
        loop_callbacks = [
            webrtc_node.spin,
            webrtc_node.publish_state
        ]
    )
    interface.start()
