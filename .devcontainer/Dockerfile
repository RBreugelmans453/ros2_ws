FROM --platform=linux/arm64 ubuntu:focal
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Update all packages
RUN apt-get update && \
    apt dist-upgrade -y && \
    apt-get install -y --no-install-recommends \
        python-is-python3 \
        gdb \
	    i2c-tools \
        curl \
        ros-humble-imu-tools \
        ros-humble-teleop-twist-joy \
        ros-humble-teleop-twist-keyboard \
        ros-humble-xacro \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-mqtt-client \
        ros-humble-joint-state-publisher \
        ros-humble-robot-state-publisher \
        ros-humble-rplidar-ros \
        libi2c-dev \
        libserial-dev \

        ros-humble-image-transport

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
