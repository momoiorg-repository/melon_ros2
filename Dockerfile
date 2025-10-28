FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]

# pkg install
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-pip vim xterm less wget python3-colcon-common-extensions python3-rosdep x11-apps \
    # Install ROS2 related packages
    ros-humble-rmw-fastrtps-cpp \
    # For Intel Realsense
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    # For navigation2
    ros-humble-nav2-bringup \
    # For moveit!
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-topic-based-ros2-control

RUN pip3 uninstall -y numpy
RUN pip3 install numpy==1.26.4
RUN pip3 install pyquaternion matplotlib transforms3d simple-pid \
 numpy-quaternion

# dds_config (For Fast DDS)
COPY ./dds_config/fastdds.xml /root/fastdds.xml

# for Actor and BT applications
# Create Colcon workspace with external dependencies
WORKDIR /
RUN mkdir -p /project/lib_ws/src
WORKDIR /project/lib_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

WORKDIR /project/lib_ws/src/pymoveit2
COPY ./project/resource/pymoveit2_setup.py setup.py

WORKDIR /project/lib_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
 && colcon build --symlink-install

WORKDIR /project
COPY ./project .

WORKDIR /root
COPY ./bin bin

# for Melon ROS2 packages
WORKDIR /root/melon_ws
COPY ./melon_ws/src ./src

# rosdep
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends curl gnupg lsb-release ca-certificates && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get update -y && \
    rosdep update --rosdistro=humble && \
    rosdep install --from-paths src --ignore-src -r -y

# Build workspace
RUN source /opt/ros/humble/setup.sh && colcon build --symlink-install

# Build arguments (default values)
ARG ROS_DOMAIN_ID=80
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}


RUN echo '# ROS2 setup' >> ~/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}' >> ~/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc && \
    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds.xml' >> ~/.bashrc && \
    echo 'source /root/melon_ws/install/setup.bash' >> ~/.bashrc && \
    echo "source /project/lib_ws/install/setup.bash" >> ~/.bashrc && \
    echo 'PATH=$PATH:/root/bin' >> ~/.bashrc

