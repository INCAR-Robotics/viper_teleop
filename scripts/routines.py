from abc import ABC, abstractmethod
import copy
import json
import math
import os
import random
import sys
import time

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def parse_routines(config: list) -> list:
    routine_list = []
    for routine_config in config:
        routine_type = routine_config['type']
        routine_params = routine_config.get('params', {})
        
        try:
            # Assumes the routine classes are in the same module
            routine_class = getattr(sys.modules[__name__], routine_type)
        except (ImportError, AttributeError) as e:
            raise ValueError(f"Could not instantiate routine class {routine_type}: {e}")
        
        # Instantiate the routine with its parameters
        try:
            routine_instance = routine_class(**routine_params)
        except Exception as e:
            raise ValueError(f"Error instantiating {routine_type}: {e}")
        
        # Add to the routine dictionary
        routine_list.append(routine_instance)
    
    return routine_list

class Routine(ABC):
    @abstractmethod
    def execute(self, bot: InterbotixManipulatorXS, logger, current_ee_pose_matrix):
        raise NotImplementedError()

class CloseAndOpenGripperRoutine(Routine):
    closed_position: float = 0.8

    def __init__(self, closed_position: float):
        self.closed_position = closed_position

    def execute(self, bot, logger, current_ee_pose_matrix):
        bot.gripper.core.robot_write_joint_command('gripper', self.closed_position)
        time.sleep(0.5)
        bot.gripper.core.robot_write_joint_command('gripper', bot.gripper.open_position)
        time.sleep(0.5)

class SetpointRoutine(Routine):
    joint_positions: list[float] = []
    
    def __init__(self, joint_positions: list[float]):
        self.joint_positions = joint_positions

    def execute(self, bot, logger, current_ee_pose_matrix):
        bot.arm.set_trajectory_time(3)
        bot.arm.set_joint_positions(self.joint_positions, moving_time=3, blocking=True)
    
class RandomRelativeXYPos(Routine):
    x_lim: float = 0
    y_lim: float = 0

    def __init__(self, x_lim: float, y_lim: float):
        self.x_lim = x_lim
        self.y_lim = y_lim

    def execute(self, bot, logger, current_ee_pose_matrix):
        bot.arm.set_trajectory_time(2)
        # current_ee_pose = bot.arm.get_ee_pose()

        target_ee_pose = current_ee_pose_matrix
        target_ee_pose[0, 3] = current_ee_pose_matrix[0,3] + random.uniform(-self.x_lim, self.x_lim)
        target_ee_pose[1, 3] = current_ee_pose_matrix[1,3] + random.uniform(-self.y_lim, self.y_lim)
        
        current_joint_positions = bot.arm.get_joint_positions()
        succes = bot.arm.set_ee_pose_matrix(
            target_ee_pose,
            custom_guess=current_joint_positions,
            moving_time=2,
            blocking=True
        )
        logger.info(f"Routine succesful? {succes}")

class RandomXYPos(Routine):
    x_upper: float = 0.20
    x_lower: float = 0.15
    y_upper: float = 0.5
    y_lower: float = -0.5
    z: float = 0.15

    def __init__(self, x_upper, x_lower, y_upper, y_lower, z, seed):
        self.x_upper = x_upper
        self.x_lower = x_lower
        self.y_upper = y_upper
        self.y_lower = y_lower
        self.z = z
        random.seed(seed)

    def execute(self, bot, logger, current_ee_pose_matrix):
        target_pose = copy.copy(current_ee_pose_matrix)
        target_pose[0, 3] = random.uniform(self.x_lower, self.x_upper)
        target_pose[1, 3] = random.uniform(self.y_lower, self.y_upper)
        target_pose[2, 3] = self.z

        bot.arm.set_trajectory_time(3)
        current_joint_positions = bot.arm.get_joint_positions()
        succes = bot.arm.set_ee_pose_matrix(
            target_pose,
            custom_guess=current_joint_positions,
            moving_time=3,
            blocking=True
        )
        logger.info(f"Current pos: {current_ee_pose_matrix[0,3]}, {current_ee_pose_matrix[1,3]}, {current_ee_pose_matrix[2,3]}")
        logger.info(f"Target pos: {target_pose[0,3]}, {target_pose[1,3]}, {target_pose[2,3]}")
        logger.info(f"Routine succesful? {succes}")


class DynamicSetpointRoutine(Routine):
    config_file: str = ""
    key: str = ""

    def __init__(self, cfg_file, key):
        self.config_file = cfg_file
        self.key = key

    def execute(self, bot, logger, current_ee_pose_matrix):
        with open(self.config_file, 'r') as f:
            data = json.load(f)
        joint_positions = data[self.key]

        bot.arm.set_trajectory_time(5)
        bot.arm.set_joint_positions(joint_positions, moving_time=5, blocking=True)

class SetSetpointRoutine(Routine):
    config_file: str = ""
    key: str = ""

    def __init__(self, cfg_file, key):
        self.config_file = cfg_file
        self.key = key

    def execute(self, bot, logger, current_ee_pose_matrix):
        joint_pos = bot.arm.get_joint_positions()

        with open(self.config_file, 'r') as f:
            data = json.load(f)
            data[self.key] = joint_pos

        os.remove(self.config_file)
        with open(self.config_file, 'w') as f:
            json.dump(data, f, indent=4)

class RemoveCapRoutine(Routine):
    n_rotations = 2
    def execute(self, bot, logger, current_ee_pose_matrix):
        bot.arm.set_trajectory_time(1)

        # Start by dropping half a centimeter because it was trained on that
        # before dropping got fixed
        # target_ee_pose = copy.copy(current_ee_pose_matrix)
        # target_ee_pose[2, 3] = current_ee_pose_matrix[2,3] - 0.005

        # current_joint_positions = bot.arm.get_joint_positions()
        # bot.arm.set_ee_pose_matrix(
        #     target_ee_pose,
        #     custom_guess=current_joint_positions,
        #     moving_time=0.5,
        #     blocking=True
        # )

        # Actual routine
        bot.arm.set_trajectory_time(1)

        initial_joint_position = bot.arm.get_joint_positions()[5]
        target_joint_position = initial_joint_position + math.pi/4
        bot.arm.set_single_joint_position("wrist_rotate", target_joint_position, moving_time=1, blocking=True)

        for i in range(self.n_rotations):
            bot.gripper.core.robot_write_joint_command('gripper', bot.gripper.closed_position)
            time.sleep(0.2)

            target_joint_position = initial_joint_position - math.pi/4
            bot.arm.set_single_joint_position("wrist_rotate", target_joint_position, moving_time=1, blocking=True)

            bot.gripper.core.robot_write_joint_command('gripper', bot.gripper.open_position)
            time.sleep(0.2)
            if i == self.n_rotations - 1: continue

            target_joint_position = initial_joint_position + math.pi/4
            bot.arm.set_single_joint_position("wrist_rotate", target_joint_position, moving_time=1, blocking=True)


        bot.arm.set_single_joint_position("wrist_rotate", initial_joint_position, moving_time=1, blocking=True)
        bot.gripper.core.robot_write_joint_command('gripper', bot.gripper.closed_position)
        time.sleep(0.2)

        target_ee_pose = copy.copy(current_ee_pose_matrix)
        target_ee_pose[2, 3] = current_ee_pose_matrix[2,3] + 0.15

        current_joint_positions = bot.arm.get_joint_positions()
        bot.arm.set_ee_pose_matrix(
            target_ee_pose,
            custom_guess=current_joint_positions,
            moving_time=1,
            blocking=True
        )

class ShakeZRoutine(Routine):
    n_shakes = 3
    shake_amplitude = 0.015
    home_position = [0.8728, -0.4847, 0.3267, -0.8943, 1.8622, 1.5309]

    def execute(self, bot, logger, current_ee_pose_matrix):
        target_pose_low = copy.copy(current_ee_pose_matrix)
        target_pose_low[2, 3] = target_pose_low[2, 3] - self.shake_amplitude/2
        target_pose_high = copy.copy(current_ee_pose_matrix)
        target_pose_high[2, 3] = target_pose_high[2, 3] + self.shake_amplitude/2

        bot.arm.set_trajectory_time(0.02)
        current_joint_positions = bot.arm.get_joint_positions()

        for i in range(self.n_shakes):
            bot.arm.set_ee_pose_matrix(
                target_pose_high,
                custom_guess=current_joint_positions,
                moving_time=0.02,
                blocking=True
            )

            bot.arm.set_ee_pose_matrix(
                target_pose_low,
                custom_guess=current_joint_positions,
                moving_time=0.02,
                blocking=True
            )

        bot.arm.set_ee_pose_matrix(
            current_ee_pose_matrix,
            custom_guess=current_joint_positions,
            moving_time=0.02,
            blocking=True
        )

        bot.arm.set_trajectory_time(3)
        bot.arm.set_joint_positions(self.home_position, moving_time=3, blocking=True)
        
class FillContainerRoutine(Routine):
    # Each arm runs its own instance (configured per arm): pose_1 -> pose_2 -> optional
    # gripper close -> pose_1. Both arms use identical timings so that, when triggered
    # together, they stay in lockstep without any explicit cross-arm synchronisation.
    pose_1: list[float] = []
    pose_2: list[float] = []
    close_gripper: bool = False
    moving_time: float = 3.0
    gripper_time: float = 0.5

    def __init__(self, pose_1: list[float], pose_2: list[float], close_gripper: bool = False,
                 moving_time: float = 3.0, gripper_time: float = 0.5):
        self.pose_1 = pose_1
        self.pose_2 = pose_2
        self.close_gripper = close_gripper
        self.moving_time = moving_time
        self.gripper_time = gripper_time

    def execute(self, bot, logger, current_ee_pose_matrix):
        bot.arm.set_trajectory_time(self.moving_time)
        bot.arm.set_joint_positions(self.pose_1, moving_time=self.moving_time, blocking=True)
        bot.arm.set_joint_positions(self.pose_2, moving_time=self.moving_time, blocking=True)

        if self.close_gripper:
            bot.gripper.core.robot_write_joint_command('gripper', bot.gripper.closed_position)
        # The arm that doesn't close its gripper waits too, to stay in step with the other one
        time.sleep(self.gripper_time)

        bot.arm.set_joint_positions(self.pose_1, moving_time=self.moving_time, blocking=True)
