# Start local registry:

#     docker run -d -p 5000:5000 --restart=always --name registry registry:2

# Build image and push to gram registry:

#     docker build --file Dockerfile -t gram:5000/spherorvr-ros2:arm . --push

#     docker build --file Dockerfile -t gram:5000/spherorvr-ros2:amd . --push

# Build multi-platform image and push to local registry:

#     docker buildx build --platform linux/amd64,linux/arm64 --file Dockerfile -t gram:5000/spherorvr-ros2:latest . --push

# Bringup rvr:

#     docker run -it --rm --network=host --privileged --name=rvr gram:5000/spherorvr-ros2:amd  

# Access bash prompt on running container:

#     docker exec -it remobot /bin/bash

FROM ros:iron-ros-base-jammy
# USER root

# Suppress all interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

RUN sudo apt update -y && \  
    sudo apt install -y \
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
RUN mkdir -p $ROS_WS
WORKDIR $ROS_WS

# Copy sphero directory from host into container workspace
COPY src/ $ROS_WS/src/
# RUN vcs import < $ROS_WS/src/sphero_other.repos
RUN pip3 install sphero-sdk
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN colcon build

# Tell container that UI output goes to host X11 server
# Note: Host must execute 'xhost +local:' command BEFORE rviz starts    
ENV DISPLAY=:0
ENV ROS_MASTER_URI=http://localhost:11311/

COPY  ../ros_entrypoint.bash .
RUN chmod +x ./ros_entrypoint.bash
ENTRYPOINT ["./ros_entrypoint.bash"]
# Following executes at <exec "$@"> in entrypoint file
CMD ["/bin/bash"]
