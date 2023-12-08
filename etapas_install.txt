1 - Vscode
2 - Git sudo apt install git
3 - Clone Repositorio


4- Seguindo Tutorial do ROS :
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

5- Configurações de ambiente: 
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc

6- Bibliotecas auxiliates:
sudo apt update
apt install -y nano \
vim \
iputils-ping \
iproute2 \
wget \
libserial-dev


7- Slam_Toolbox
sudo apt install ros-humble-slam-Slam_Toolbox

8- Nav2
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

9- Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros2-control

7- Colcon build
cd ~/dev_ws
colcon build