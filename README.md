This example Dockerfile is from the following tutorial: xxxx.

It borrows heavily from [Allison Thackston's Dockerfile repo](https://github.com/athackst/dockerfiles) and the [OSRF Docker images](https://github.com/osrf/docker_images) were also used for inspiration.



RUN cd dev_ws
RUN rosdep update --rosdistro=$ROS_DISTRO
RUN apt-get update
RUN rosdep install --from-paths src --ignore-src -r -y
