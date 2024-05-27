#!/bin/bash
echo "Setting up environment"
apt update
apt install pip -y
pip install pipenv
pipenv --python `which python3` install

# Installing scenario-player reqs.
echo "Installing pip packages for L2 agents"
pip install -r ./requirements.txt
