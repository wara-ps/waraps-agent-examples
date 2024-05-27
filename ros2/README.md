# Introduction 
This repo contains a ROS2 humble workspace (ros2_simulated_agent). 
The workspace is an example of how to connect to the core systems MQTT through a ROS2 node and publish sensor values and heartbeat as an L1 agent in the Core System.

# Getting started

Open project in Ubuntu 22.04 environment.
Wether the code is run with ros2 or docker, it is done so inside the workspace, so start by:

```cd ros2_simulated_agent```

####Default Broker:
Connects to 'broker.waraps.org' on port 8883 with TSL enabled  
To change this edit the file ```'src/simulated_agent_l1_bringup/config/params.yaml'```. In that file you also need to provide the username and password for the broker.

Install requirements with pip using python3-pip

# Using Ros2
Install ROS2 humble and colcon and build packages by running:

```colcon build```

Then source the local installation by running:

```source install/setup.bash```

Source the installation:

```source /opt/ros/humble/setup.bash```

Finally launch the two nodes by running:

```ros2 launch simulated_agent_l1_bringup simulated_agent_l1_launch.py```

# Using docker

If you use docker to run the code, ROS2 does not have to be installed beforehand.
To build the docker image run:

```docker build -t ros2_agent:latest```

and run the code by:

```docker-compose up```
