FROM osrf/ros:humble-desktop-full as base
SHELL ["/bin/bash", "-c"]

#Make ROS2 workspace
RUN mkdir -p /ros2_simulated_agent/src
WORKDIR /ros2_simulated_agent

RUN apt-get update && apt-get install -y \
     python3-pip \
     python3-colcon-common-extensions 

#Install needed python packages
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt
RUN pip install setuptools==58.2.0

RUN source /opt/ros/humble/setup.bash 
COPY ./src ./src

#Build packages
RUN colcon build --symlink-install

