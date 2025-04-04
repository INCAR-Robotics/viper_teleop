# Installation instructions:
1. Install the interbotix packages by running their install scripts. This will also install all required dependencies such as ROS
```bash
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```

2. A workspace called `/interbotix_ws` has now been created. Navigate to to `/interbotix_ws/src` and clone this repo. Then build the workspace
```bash
git clone https://github.com/INCAR-Robotics/viper_teleop.git
cd ..
colcon build
```

3. Ensure that the right udev rules are setup by following the following steps:
- Plug in the left robot only and check which port the robot is bound to (you can use `ls /dev`), e.g. `ttyUSB0`
- Run `udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial` to obtain the serial number. Use the first one that shows up, the format should look similar to `FT6S4DSP`.
- Run `sudo code /etc/udev/rules.d/99-fixed-interbotix-udev.rules` and add the following line:
``` shell
SUBSYSTEM=="tty", ATTRS{serial}=="<serial number here>", ENV{ID_MM_DEVICE_IGNORE}="1", ATTR{device/latency_timer}="1", SYMLINK+="left"
```
- Repeat for the right robot (but now setting the `SYMLINK` to `"right"`)
- To apply the changes, run `sudo udevadm control --reload && sudo udevadm trigger`
- To check that all went well plug in both robots now and run `ls /dev`. you should see `ttyDXL_left` and `ttyDXL_right` in the list.

# Usage
> [!CAUTION]
> The robots do not have breaks on the motors, therefore, make sure to ALWAYS return the robot to sleep position before shutting down the drivers! Otherwise, the robot will collapse.

> [!CAUTION]
> For some reason, the drivers remember the time_to_goal of the last command set. This means that if teleop crashes midway, the cached time_to_goal is very low. DON'T use the rviz interface to send the robot home or to sleep when this happens, as the robot will move at extremely high speeds. Instead, run the python script `go_to_sleep_slowly.py`

Open two terminals and make sure to source the workspace

1. Start the drivers for the robots:
```bash
ros2 launch viper_teleop dual_arm.launch.py
```

2. Start the teleop nodes and WebRTC connection using the following launch file:
```bash
ros2 launch viper_teleop teleop.launch.py
```

# TODO's
- First off make the ee controller have proper rotations
- Make IP and port launch arguments
- Make single arm versions
- Make `go_to_sleep_slowly.py` accessible with `ros2 run`