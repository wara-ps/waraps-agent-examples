#Python agent

##Introduction
This repo contains code to create virtual python agents which can be used on the 2022 Arena map and the Integration map. 

Note that the code influencing the python agents behaviour make it act as a drone would

#####Install Dependencies:
##Getting started
####Install:
To run the code for the python agents you need certain libraries which you can install through the command:
```pipenv install -r requirements.txt```

#####Prerequisites:  
If you don't use Python 3.10 or newer, you need to install 'pip' separately using the command ```python get-pip.py``` 
as well as 'pipenv' using ```pip3 install pipenv```

#####Run:  
```pipenv run python main.py```
####Code edits:
Note that you will have to fill in values in the '.env file' for each agent folder, before you can run the code for the agent in question.

### Create .env file
take the env-template file and make a .env out of it.

##Build and run
To run one python agent run the ```pipenv run python main.py``` (or docker, see below) and make sure that the ```python -u ./main.py"``` line 
in "docker-compose.yml" is not commented out. It will generate one pythonagent with a randomized name starting with pa_

To get more than one agent change comment out the line mentioned above and activate the ```python -u bulk_create.py -u 4 -n lx_agent``` 
line instead. Note that the number determines the number of agents that should be created and the string after -n 
determines the name of the agents, followed by "_(instance number)" so lx_agent_1, lx_agent_2, lx_agent_3 ect.

If you choose to only write ````python -u bulk_create.py -u 4```` in the "docker-compose.yml", you will create 4 python agents with randomized names starting with "pa_"


####Default Broker:
Connects to 'localhost' on port 1883 with TSL disabled, no username or password required
To change broker address, you can edit the values in the file '.env' file  


 #NOW WITH DOCKER 🙌
 You can also run the code by using ```docker-compose up``` OBS! Can't be used on localhost!