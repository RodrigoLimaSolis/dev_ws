## Instalação do ROS2 na Raspberry PI

### 1. Baixando VS-Code.
   ```
   sudo apt update
   sudo apt install code
   ```

### 2. Instalando Docker.
   Seguir os tutoriais do link oficial: [Install Docker Engine on Ubuntu | Docker Docs](https://docs.docker.com/engine/install/ubuntu/)
   ```
   # Add Docker's official GPG key:
   sudo apt-get update
   sudo apt-get install ca-certificates curl gnupg
   sudo install -m 0755 -d /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
   sudo chmod a+r /etc/apt/keyrings/docker.gpg

   # Add the repository to Apt sources:
   echo \
   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
   $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   sudo apt-get update

   #Install the Docker packages.
   sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   
   #Verify that the Docker Engine installation is successful by running the hello-world image.
   sudo docker run hello-world
   ```
   
      
### 3. Pós-instalação do Docker.
   Seguir os tutoriais do link oficial: [Linux post-installation steps for Docker Engine | Docker Docs](https://docs.docker.com/engine/install/linux-postinstall/)
   ```
   sudo groupadd Docker 
   sudo usermod -aG docker $USER
   newgrp docker     
   ```

### 4. Baixando repósitorio da SOLIS.
   ```
   cd ~/
   git clone -b raspberrypi https://github.com/RodrigoLimaSolis/dev_ws.git
   ```

### 5. Buildando a imagem.
   ! Essa etapa dependendo do processamento e da internet pode demorar bastante - 600s levou o último build
   ```
   docker image build -t ros2_humble ~/dev_ws/
   ```

### 6. Crie um container com base nessa imagem.
   No código a seguir temos que referenciar em quais portas USB queremos ter acesso dentro container, portanto é necessário saber em quais portas elas estão conectadas em nossa raspberry.
   * A placa de controle: `/dev/ttyACM0`
   * O Lidar: `/dev/ttyUSB0`

   ```
   docker run -it --user ros --network=host --ipc=host -v ~/dev_ws/dev_ws:/dev_ws -v /tmp/.X11-unix:/tmp/.X11-unix:rw -env=DISPLAY   --device=/dev/ttyACM0  --device=/dev/ttyUSB0  --name ros2_image ros2_humble  
   ```

### 7. Executando container.
   ```
   docker exec -it ros2_image /bin/bash
   ```
### 8. Hello World no ROS2.
   Para verificar se tudo foi corretamente instalado execute dentro do container:
   ```
   ros2 run demo_nodes_cpp talker
   ```

## Buildando os pacotes do ros2_control

### 1. Entre em seu workspace dentro do container.
   ```
   cd dev_ws 
   ```

### 2. Baixando dependencias.
   ```
   rosdep update --rosdistro=$ROS_DISTRO  
   sudo apt-get update  
   rosdep install --from-paths src --ignore-src -r -y
   ```

### 3. Builde.
   ```
   colcon build --symlink-install
   ```

### 4. Tornando os scripts executáveis.
   ```
   chmod +x /dev_ws/script/lidar.sh 
   chmod +x /dev_ws/script/ros2_control.sh 
   chmod +x /dev_ws/script/teleop.sh 
   ```


## Executando os Pacotes
### 1.	Ros2_control.
```
ros2 launch sobot_drive diffbot.launch.py
/script/ros2_control.bash
```
Se todos os passos foram seguidos corretamente, o sobot irá piscar em verde duas vezes e ficar com a cor verde. E uma tela irá aparecer com um monte informação sendo correndo sobre ela.


### 2.	 Lidar.
```
ros2 launch sobot_drive rplidar.launch.py
/script/lidar.sh
```
Para pausar e iniciar o motor (após o comando acima ser executado)
```
ros2 service call /start_motor std_srvs/srv/Empty {} #Inicia o lidar
ros2 service call /stop_motor std_srvs/srv/Empty {} #Pausa o lidar
```


### 3.	Teleop.
```
ros2 run teleop_twist_keyboard_sobot teleop_twist_keyboard_sobot --ros-args -r /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```

Para vizualizar os comandos enviados (Sem aceleração e com aceleração, respectivamente).

```
ros2 topic echo /diffbot_base_controller/cmd_vel_unstamped
ros2 topic echo /diffbot_base_controller/cmd_vel_out 
```
