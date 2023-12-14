## Instalação do ROS2 no Deskop(Linux)

### 1. Baixando VS-Code.
   ```
   sudo apt update
   sudo apt install code
   ```

### 2.	Git.
```
sudo apt install git
```

### 3.	Clonar Repositório.
```
git clone -b desktop https://github.com/RodrigoLimaSolis/dev_ws.git
```

### 4.	ROS2 Humble.
Link da documentação de referência: [Ubuntu Debian packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
```
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```


### 5.	Configurações do Ambiente ROS2
Link da documentação de referência: [Configuring environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/dev_ws/install/setup.bash" >> ~/.bashrc
```

### 6.	Bibliotecas Auxiliares
```
sudo apt update
apt install -y nano \
vim \
iputils-ping \
iproute2 \
wget \
libserial-dev
```

#### 7.	Slam_Toolbox
```
sudo apt install ros-humble-slam-Slam_Toolbox

#After Installation
cp -r ~/dev_ws/src/sobot_drive/description/meshes/ ~/.gazebo/models/
```

### 8.	 Nav2
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### 9.Gazeboo
```
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-ros2-control
```

### 10. Colcon build
```
cd ~/dev_ws
colcon build
```

### 11.	Hello World no Desktop
```
ros2 run demo_nodes_cpp talker
```

## Executando a Navegação Autônoma
![](imgs\nav.png)
### 1.	Script
Automaticamente todos os programas serão iniciados.
```
~/dev_ws/script/desktop.sh
```

### 2.	Rviz2
```
rviz2 -d ~/dev_ws/src/sobot_drive/bringup/config/default.rviz
```

Será iniciado o rviz2 com todas os parâmetros e plugins configurados;

### 3.	Slam_toolbox
```
ros2 launch sobot_drive online_async_launch.py
```

O slam_toolbox será iniciado com todas nossas configurações.

### 4.	 NAV2
```
ros2 launch sobot_drive navigation_launch.py
```

## Executando a simulação no Gazebo
![](imgs\gazebo_horz_pos_topbar.svg)
### 1.	Script
```
~/dev_ws/script/gazebo.sh
```

Automaticamente todos os programas serão iniciados.
### 2.	Rviz2
```
rviz2 -d ~/dev_ws/src/sobot_drive/bringup/config/default.rviz use_sim_time:=true	
```

Será iniciado o rviz2 com todas os parâmetros e plugins configurados + o tempo de simulação que será definido como tempo padrão;

### 3.	Gazebo
```
ros2 launch sobot_gazebo launch_sim.launch.py world:=./src/sobot_gazebo/worlds/world1.world use_sim_time:=true"
```

O processo para iniciar o gazebo pode demorar alguns segundos, por este motivo foi adicionado um tempo de espera de 10 segundos até iniciar o próximo comando. 

### 4.	Slam_ToolBox
```
ros2 launch sobot_gazebo online_async_launch.py use_sim_time:=true
```

O slam_toolbox será iniciado com todas as configurações para rodar no ambiente virtual.
### 5.	 NAV2
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

Vamos ter o nav2 iniciado e configurado para rodar com base no ambiente de simulação
### 6.	Teleop
```
ros2 run teleop_twist_keyboard_sobot teleop_twist_keyboard_sobot --ros-args -r /cmd_vel:=/ diff_cont/cmd_vel_unstamped
```

Para vizualizar os comandos enviados (Sem aceleração e com aceleração, respectivamente).
```
ros2 topic echo /diff_cont/cmd_vel_unstamped
ros2 topic echo / diff_cont/cmd_vel_out 
```
