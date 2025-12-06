FROM ros:humble-ros-base

# install ros package
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-foxglove-bridge \
    python3-pip \
    iproute2 \
    can-utils && \
    rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip install --upgrade pip
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# setup colcon extensions
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# create a workspace directory
RUN mkdir -p ~/ros2_ws/src
RUN mkdir -p ~/ros2_ws/.vscode
RUN mkdir -p ~/ros2_ws/third_party

# import third party repos into workspace
COPY third_party.repos /tmp/third_party.repos
RUN vcs import ~/ros2_ws/third_party < /tmp/third_party.repos

# Ignore packages that are not needed / cause trouble in the SDK 
RUN touch ~/ros2_ws/third_party/ros_odrive/odrive_ros2_control/COLCON_IGNORE || true
RUN touch ~/ros2_ws/third_party/ros_odrive/odrive_botwheel_explorer/COLCON_IGNORE || true

# auto source ROS setup.bash
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# auto source workspace overlay if one exists
RUN echo "\n if [ -f ~/ros2_ws/install/setup.bash ]; then\n source ~/ros2_ws/install/setup.bash\n fi\n" >> ~/.bashrc

WORKDIR /root/ros2_ws
