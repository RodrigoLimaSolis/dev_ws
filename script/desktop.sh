gnome-terminal --tab --command="rviz2"
gnome-terminal --tab --command="ros2 launch sobot_drive online_async_launch.py"
gnome-terminal --tab --command="ros2 run twist_mux twist_mux --ros-args --params-file /home/solis/dev_ws/src/sobot_gazebo/config/twist_mux.yaml -r cmd_vel_out:=diffbot_base_controller/cmd_vel_unstamped"
gnome-terminal --tab --command="ros2 launch sobot_drive navigation_launch.py"
