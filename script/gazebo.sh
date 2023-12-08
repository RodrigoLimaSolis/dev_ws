gnome-terminal --tab --command="ros2 launch sobot_gazebo launch_sim.launch.py world:=./src/sobot_gazebo/worlds/world1.world use_sim_time:=true"
sleep 10
gnome-terminal --tab --command="sudo cd /home/solis/dev_ws && ros2 run twist_mux twist_mux --ros-args --params-file /home/solis/dev_ws/src/sobot_gazebo/config/twist_mux.yaml -r cmd_vel_out:=/diff_cont//cmd_vel_unstamped"
sleep 2
gnome-terminal --tab --command="ros2 launch sobot_gazebo online_async_launch.py use_sim_time:=true"
sleep 2
gnome-terminal --tab --command="ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true"
sleep 2
gnome-terminal --tab --command="rviz2 use_sim_time:=true"
