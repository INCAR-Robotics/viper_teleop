from interbotix_common_modules.common_robot.robot import (
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import math
import time

from tf_transformations import euler_from_matrix, euler_matrix
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    right_bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
    )

    np.set_printoptions(suppress=True)
    robot_startup()
    right_bot.arm.go_to_home_pose(moving_time=3, blocking=True) 
    # right_bot.arm.set_ee_pose_components(
    #     x = 0.3,
    #     y = 0,
    #     z = 0.3,
    #     roll = 0,
    #     pitch = 1.57,
    #     yaw = 0,
    #     moving_time=3,
    #     blocking=True
    # )
    # time.sleep(2)
    
    # print(right_bot.arm.get_ee_pose())
    roll=0
    pitch=1.57
    yaw = 0
    r = R.from_euler('xyz', [roll, pitch, yaw])
    T_mat = np.identity(4)
    T_mat[:3, :3] = euler_matrix(roll, pitch, yaw)[:3, :3]
    # T_mat[:3, :3] = [[ 0.0007,   -0.5646,    0.8253],
    #     [0.0004,    0.8253,    0.5646],
    #     [-1.0000,         0,    0.0008]]
    T_mat[:3, 3] = [0.3, 0, 0.3]
    print("T_mat\n", T_mat)

    right_bot.arm.set_ee_pose_matrix(
        T_mat,
        moving_time=3,
        blocking=True
    )

    time.sleep(2)
    robot_rot = right_bot.arm.get_ee_pose()[:3, :3]

    roll=0.6
    pitch=0
    yaw = 0
    r = R.from_euler('xyz', [roll, pitch, yaw])
    T_mat = np.identity(4)
    # T_mat[:3, :3] = r.as_matrix()
    print( euler_matrix(roll, pitch, yaw)[:3, :3])
    print( robot_rot)
    print( euler_matrix(roll, pitch, yaw)[:3, :3] @ robot_rot)

    T_mat[:3, :3] = euler_matrix(roll, pitch, yaw)[:3, :3] @ robot_rot
    T_mat[:3, 3] = [0.3, 0, 0.3]


    # T_mat[:3, :3] =     [[0.0008,    0.5646,    0.8253],
    #      [0,    0.8253,   -0.5646],
    #     [-1.0000,    0.0004,    0.0007]]
    print("T_mat\n", T_mat)

    right_bot.arm.set_ee_pose_matrix(
        T_mat,
        moving_time=3,
        blocking=True
    )

    time.sleep(2)

    roll=0
    pitch=0
    yaw = 0.6
    r = R.from_euler('xyz', [roll, pitch, yaw])
    T_mat = np.identity(4)
    # T_mat[:3, :3] = r.as_matrix()
    T_mat[:3, :3] = euler_matrix(roll, pitch, yaw)[:3, :3] @ robot_rot
    T_mat[:3, 3] = [0.3, 0, 0.3]


    # T_mat[:3, :3] =     [[0.0008,    0.5646,    0.8253],
    #      [0,    0.8253,   -0.5646],
    #     [-1.0000,    0.0004,    0.0007]]
    print("T_mat\n", T_mat)

    right_bot.arm.set_ee_pose_matrix(
        T_mat,
        moving_time=3,
        blocking=True
    )




    time.sleep(100)
    # amp = 0.5
    # freq = 0.2
    # t = 0
    # while True:
    #     robot_rot = euler_from_matrix(right_bot.arm.get_ee_pose()[:3, :3])
    #     print(right_bot.arm.get_ee_pose())
    #     s = amp*math.sin(t*freq*2*math.pi)
    #     right_bot.arm.set_ee_pose_components(
    #         x = 0.3,
    #         y = 0,
    #         z = 0.3,
    #         roll = 0,
    #         pitch = 1.57,
    #         yaw = s,
    #         moving_time=0.2,
    #         blocking=False
    #     )
    #     time.sleep(0.2)
    #     t += 0.2

    right_bot.arm.go_to_sleep_pose(moving_time=3, blocking=True)
    robot_shutdown()


if __name__ == '__main__':
    main()