from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    get_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


def main():
    node = create_interbotix_global_node()

    left_bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        robot_name='left',
        group_name='arm',
        node=node,
        gripper_name='gripper',
    )
    right_bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        robot_name='right',
        group_name='arm',
        node=node,
        gripper_name='gripper',
    )

    robot_startup()
    left_bot.arm.go_to_sleep_pose(moving_time=5, blocking=False)    
    right_bot.arm.go_to_sleep_pose(moving_time=5, blocking=True)

    # teleop_node.py leaves the gripper motor in 'current_based_position' mode, which persists
    # on the xs_sdk driver across process restarts; restore 'pwm' mode so release()'s effort
    # value isn't misread as a raw position target.
    left_bot.gripper.core.robot_set_operating_modes('single', 'gripper', 'pwm')
    right_bot.gripper.core.robot_set_operating_modes('single', 'gripper', 'pwm')
    left_bot.gripper.release()
    right_bot.gripper.release()

    robot_shutdown()


if __name__ == '__main__':
    main()
