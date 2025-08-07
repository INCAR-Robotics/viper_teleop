from abc import ABC, abstractmethod
import math
import time

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class Routine(ABC):
    @abstractmethod
    def execute(self, bot: InterbotixManipulatorXS, logger):
        raise NotImplementedError()


class SetpointRoutine(Routine):
    joint_positions: list[float] = []
    
    def __init__(self, joint_positions: list[float]):
        self.joint_positions = joint_positions

    def execute(self, bot, logger):
        bot.arm.set_trajectory_time(5)
        bot.arm.set_joint_positions(self.joint_positions, moving_time=5, blocking=True)
    

class RemoveCapRoutine(Routine):
    n_rotations = 3
    def execute(self, bot, logger):
        current_joint_positions = bot.arm.get_joint_positions()
        bot.arm.set_trajectory_time(2)

        target_positions = current_joint_positions
        target_positions[5] = current_joint_positions[5] + math.pi / 4
        bot.arm.set_joint_positions(target_positions, moving_time=2, blocking=True)

        for i in range(self.n_rotations):
            bot.gripper.grasp(0.5)

            target_positions = current_joint_positions
            target_positions[5] = current_joint_positions[5] - math.pi / 4
            bot.arm.set_joint_positions(target_positions, moving_time=2, blocking=True)

            bot.gripper.release(0.5)

            target_positions = current_joint_positions
            target_positions[5] = current_joint_positions[5] + math.pi / 4
            bot.arm.set_joint_positions(target_positions, moving_time=2, blocking=True)

        bot.arm.set_joint_positions(current_joint_positions, moving_time=2, blocking=True)
        
        bot.gripper.grasp(0.5)

        current_ee_pose = bot.arm.get_ee_pose()
        target_ee_pose = current_ee_pose
        target_ee_pose[2, 3] = current_ee_pose[2,3] + 0.05
        bot.arm.set_ee_pose_matrix(
            target_ee_pose,
            custom_guess=current_joint_positions,
            moving_time=2,
            blocking=True
        )