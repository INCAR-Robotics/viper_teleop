from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    get_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import modern_robotics as mr
import math
import time

from tf_transformations import euler_from_matrix

def main():
    right_bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()
    right_bot.arm.go_to_home_pose(moving_time=3, blocking=True)
    # right_bot.arm.set_single_joint_position("wrist_angle", 1.57, moving_time=3, blocking=True)    
    right_bot.arm.set_ee_pose_components(
        x = 0.3,
        y = 0,
        z = 0.3,
        roll = 0,
        pitch = 1.57,
        yaw = 0,
        moving_time=3,
        blocking=True
    )

    # right_bot.arm.set_ee_pose_components(
    #     x = 0.3,
    #     y = 0,
    #     z = 0.3,
    #     roll = 0.78,
    #     pitch = 1.57,
    #     yaw = 0,
    #     moving_time=3,
    #     blocking=True
    # )

    amp = 0.5
    freq = 0.2
    t = 0
    time.sleep(2)
    while True:
        robot_rot = euler_from_matrix(right_bot.arm.get_ee_pose())
        print(robot_rot)
        s = amp*math.sin(t*freq*2*math.pi)
        right_bot.arm.set_ee_pose_components(
            x = 0.3,
            y = 0,
            z = 0.3,
            roll = s,
            pitch = 1.57,
            yaw = 0,
            moving_time=0.2,
            blocking=False
        )
        time.sleep(0.2)
        t += 0.2

    right_bot.arm.go_to_sleep_pose(moving_time=3, blocking=True)
    robot_shutdown()


if __name__ == '__main__':
    main()