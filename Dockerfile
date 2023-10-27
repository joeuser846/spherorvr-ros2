# Start registry on devhost:

#     docker run -d -p 5000:5000 --restart=always --name registry registry:2

# Build image and push to devhost registry:

#     docker build --file Dockerfile -t gram:5000/spherorvr-ros2:arm . --push

#     docker build --file Dockerfile -t gram:5000/spherorvr-ros2:amd . --push

# Build multi-platform image and push to devhost registry:

#     docker buildx build --platform linux/amd64,linux/arm64 --file Dockerfile -t gram:5000/spherorvr-ros2:latest . --push

# Bringup rvrbot:

#     docker run -it --rm --network=host --privileged --name=rvrbot gram:5000/spherorvr-ros2:arm  

#     docker run -it --rm --network=host --privileged --name=rvrbot gram:5000/spherorvr-ros2:amd

# Access bash prompt on running container:

#     docker exec -it rvrbot /bin/bash

FROM ros:iron-ros-base-jammy
# Use bash shell to enable source command
SHELL ["/bin/bash", "-c"]

# Suppress all interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install tools required at build time
RUN sudo apt update -y && \  
    sudo apt install -y \
    python3-vcstool \
    python3-pip \
    git \
# Install tools for debugging at runtime
    net-tools \
    iputils-ping \
    netcat \
    nano
      
# Build colcon workspace, create empty Sphero package, and ingest Sphero Python modules from devhost
ENV ROS_WS=/opt/ros/${ROS_DISTRO}/ros2_ws
WORKDIR ${ROS_WS}/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    ros2 pkg create \
        --build-type ament_python \
        --license Apache-2.0 \
        --node-name sphero_node \
        sphero
COPY sphero/config ./sphero/config
COPY sphero/sphero ./sphero/sphero
WORKDIR ${ROS_WS}
RUN pip3 install sphero-sdk && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

ENV ROS_MASTER_URI=http://localhost:11311/

COPY  ../ros_entrypoint.bash .
RUN chmod +x ./ros_entrypoint.bash
ENTRYPOINT ["./ros_entrypoint.bash"]
# Following executes at <exec "$@"> in entrypoint file
CMD ["/bin/bash"]
