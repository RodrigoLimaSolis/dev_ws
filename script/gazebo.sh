gnome-terminal --tab --command="ros2 launch sobot_gazebo launch_sim.launch.py world:=./src/sobot_gazebo/worlds/world1.world use_sim_time:=true"
sleep 15
gnome-terminal --tab --command="ros2 launch sobot_gazebo online_async_launch.py use_sim_time:=true"
sleep 2
gnome-terminal --tab --command="ros2 launch sobot_gazebo navigation_launch.py use_sim_time:=true"
sleep 2
gnome-terminal --tab --command="rviz2 use_sim_time:=true"
gnome-terminal --tab --command="ros2 run teleop_twist_keyboard_sobot teleop_twist_keyboard_sobot --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped"
