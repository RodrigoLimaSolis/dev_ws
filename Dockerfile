FROM arm64v8/ros:humble

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    vim \
    iputils-ping \
    iproute2 \
    wget \
    ros-humble-ros2-control \
    ros-humble-demo-nodes-py && \
    libserial-dev\
    ros-humble-xacro\
    pip\
    ros-humble-rplidar-ros\
    ros-humble-generate-parameter-library\
    && rm -rf /var/lib/apt/lists/*


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY config/entrypoint.sh /entrypoint.sh
COPY config/bashrc /home/${USERNAME}/.bashrc


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]