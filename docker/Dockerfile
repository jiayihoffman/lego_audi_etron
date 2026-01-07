FROM ros:humble-perception

ARG ROS_DISTRO=humble

# Install essential utilities
RUN apt-get update && apt-get install -y \
    vim \
    python3-pip \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-rosbridge-suite \
    bluez \
    && rm -rf /var/lib/apt/lists/*

# Create "ros" user and group
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV BOT_HOME=/home/${USERNAME}

COPY start_robot.sh ${BOT_HOME}/start_robot.sh
RUN chmod +x ${BOT_HOME}/start_robot.sh

# Copy SSH key and configure permissions
RUN mkdir -p ${BOT_HOME}/.ssh && chmod 700 ${BOT_HOME}/.ssh
COPY id_rsa ${BOT_HOME}/.ssh/id_rsa
RUN ssh-keyscan github.com >> ${BOT_HOME}/.ssh/known_hosts

RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -d ${BOT_HOME} \
  && chown -R ${USER_UID}:${USER_GID} ${BOT_HOME}
    
RUN usermod -a -G dialout ${USERNAME}

# the rest of the command run as ros
USER ros

# Create the ROS 2 workspace
RUN mkdir -p ${BOT_HOME}/lego_ws/src 

# download the rplidar source code
RUN cd ${BOT_HOME}/lego_ws/src \
  && git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

RUN cd ${BOT_HOME}/lego_ws/src \
  && git clone git@github.com:jiayihoffman/audi_etron.git

# Source ROS 2 env and build the rplidar, and audi_etron hardware.
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
    && cd ${BOT_HOME}/lego_ws \
    && colcon build --symlink-install"

# config ros2 environment
RUN echo "export ROS_DOMAIN_ID=4" >> ~/.bashrc \
  && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
  && echo "source ${BOT_HOME}/lego_ws/install/setup.bash" >> ~/.bashrc

# install python dependencies
COPY requirements.txt ${BOT_HOME}/requirements.txt
RUN python3 -m pip install --user --no-cache-dir --no-compile -r ${BOT_HOME}/requirements.txt
    
# Set default working directory
WORKDIR ${BOT_HOME}/lego_ws

CMD ["/bin/bash", "-c", "${BOT_HOME}/start_robot.sh"]
