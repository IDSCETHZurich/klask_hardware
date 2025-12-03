FROM ros:humble-ros-base

# install ros package
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    python3-colcon-common-extensions \
    python3-colcon-mixin && \
    rm -rf /var/lib/apt/lists/*

# TODO: test if the colcon_cd works without these lines
# setup colcon_cd
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ~/.bashrc

# create a workspace directory
RUN mkdir -p ~/ros2_ws/src