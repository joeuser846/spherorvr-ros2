# Start local registry:

#     docker run -d -p 5000:5000 --restart=always --name registry registry:2

# Build arm64 image on Raspberry Pi and push to gram registry:

#     docker build --file Dockerfile -t gram:5000/spherorvr-ros2:latest . --push

# Build arm64 image on x86 laptop and push to local registry:

#     docker buildx build --file Dockerfile --platform linux/arm64 -t gram:5000/spherorvr-ros2:latest . --push

# Bringup rvr:

#     docker run -it --rm --network=host --privileged --name=rvr gram:5000/spherorvr-ros2:latest  

# Access bash prompt on running container:

#     docker exec -it remobot /bin/bash

FROM ros:iron-ros-base-jammy
USER root

# Suppress all interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -y && \  
    apt install -y \
    python3-vcstool \
    python3-pip \
    git \
# Install tools for debugging comms at runtime
    net-tools \
    iputils-ping \
    netcat \
    nano
      
# Set location of our container's catkin workspace
ENV ROS_WS /opt/ros/$ROS_DISTRO/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# Copy sphero directory from host into container workspace
COPY sphero/ $ROS_WS/src/sphero/
# RUN vcs import < $ROS_WS/src/sphero/sphero_other.repos
RUN pip3 install sphero-sdk
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN colcon build

# Tell container that UI output goes to host X11 server
# Note: Host must execute 'xhost +local:' command BEFORE rviz starts    
ENV DISPLAY=:0
ENV ROS_MASTER_URI=http://localhost:11311/

# When container starts execute our ros_entrypoint.sh script
# COPY ros_entrypoint.sh $ROS_WS/
# ENTRYPOINT [ "bash", "ros_entrypoint.sh" ]