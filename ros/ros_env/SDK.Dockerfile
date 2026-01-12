FROM ros:humble-ros-base
ARG OVERLAY_WS=/opt/ros/klask_ws

# install ros package
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    gdb \
    xterm \
    python3-pip \
    python3-opencv \
    iproute2 \
    can-utils \
    clang-format && \
    rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip install --upgrade pip
COPY ros_env/res/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# setup colcon extensions
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# create a workspace directory
RUN mkdir -p $OVERLAY_WS/src
RUN mkdir -p $OVERLAY_WS/.vscode
RUN mkdir -p $OVERLAY_WS/third_party

# import third party repos into workspace
COPY ros_env/res/third_party.repos /tmp/third_party.repos
RUN vcs import $OVERLAY_WS/third_party < /tmp/third_party.repos
# Ignore packages that are not needed / cause trouble in the SDK 
RUN touch $OVERLAY_WS/third_party/ros_odrive/odrive_ros2_control/COLCON_IGNORE || true
RUN touch $OVERLAY_WS/third_party/ros_odrive/odrive_botwheel_explorer/COLCON_IGNORE || true

# auto source ROS setup.bash
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# auto source workspace overlay if one exists
RUN echo "\n if [ -f $OVERLAY_WS/install/setup.bash ]; then\n source $OVERLAY_WS/install/setup.bash\n fi\n" >> ~/.bashrc

WORKDIR $OVERLAY_WS