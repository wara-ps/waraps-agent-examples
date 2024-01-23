# Introduction
This repository implements simple agent examples for the Core system in C++11.

# Getting Started
The agent can be run using either VS Code or Docker.

**Docker**  
Set up your MQTT connection in the .env file and command-line options in docker-compose.yml.
To compile and run the agent, run

    docker-compose up

**VS Code**  
Compilation requires the Paho C and C++ MQTT libraries to be installed (in that order). Paho installation on Linux can be done by running

    sudo bash install_requirements.sh
Environment variables and command line arguments are set in launch.json.

**Command line variables**  
-n, --name: set the base name for every agent  
-u, --units: set number of agents to create
