source install/setup.bash

ros2 run teleop_twist_keyboard_sobot teleop_twist_keyboard_sobot --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
