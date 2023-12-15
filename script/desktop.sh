gnome-terminal --tab --command="rviz2 -d /home/solis/dev_ws/src/sobot_drive/bringup/config/default.rviz"
gnome-terminal --tab --command="ros2 launch sobot_drive online_async_launch.py"
gnome-terminal --tab --command="ros2 launch sobot_drive navigation_launch.py"
