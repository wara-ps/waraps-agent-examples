#Python agent

##Introduction
This repo contains code to create virtual python agents which can be used on the 2022 Arena map and the Integration map. 

Note that the code influencing the python agents behaviour make it act as a drone would

##Getting started
####Install:
To run the code for the python agents you need certain libraries which you can install through the command:
```pip install -r requirements.txt```

####Code edits:
Note that you will have to fill in username and password for the mqtt broker, in ```secrets.py```, before you can run the code.

If you need a secure TSL-connection, edit the class "MqttConfig" in ````config.py````

##Build and test
To run one python agent run the ```Python main.py``` and make sure that the ```python -u ./main.py"``` line 
in "docker-compose.yml" is not commented out.

To get more than one agent change comment out the line mentioned above and activate the ```python -u bulk_create.py -u 4 -n python_agent``` 
line instead. Note that the number determines the number of agents that should be created and the string after -n 
determines the name of the agents, followed by "_(instance number)" so python_agent_1, python_agent_2, python_agent_3 ect.

If you choose to only write ````python -u bulk_create.py -u 4```` you will create 4 pythonagents with randomized names starting with "pa_"

