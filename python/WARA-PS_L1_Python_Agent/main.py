import time
from agent import Agent
import traceback, argparse
from classes import AgentConfig, MqttConfig
def main():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-n", "--name", help="Give the agents a name")
        parser.add_argument("-u", "--units", help="Input how many agents to Create")
        args = parser.parse_args()

        if args.name:
            AgentConfig.NAME = args.name
            MqttConfig.WARAPS_TOPIC_BASE =f"waraps/unit/{AgentConfig.DOMAIN}/{AgentConfig.SIM_REAL}/{args.name}"

        my_agent = Agent()

        # Main loop
        rate: float = 1.0 / my_agent.logic.rate
        while True:
            my_agent.send_heartbeat()
            my_agent.send_sensor_info()
            my_agent.send_position()
           # my_agent.send_direct_execution_info()
            
            time.sleep(rate)

    except Exception as e:
        print(traceback.format_exc())

if __name__ == "__main__":
    main()
