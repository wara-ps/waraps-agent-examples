import argparse
import time
import traceback
from agent import Agent
from classes import AgentConfig, MqttConfig


def main():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-n", "--name", help="Give the agents a name")
        parser.add_argument("-u", "--units", help="Input how many agents to create")
        args = parser.parse_args()

        if args.name:
            AgentConfig.NAME = args.name
            MqttConfig.WARAPS_TOPIC_BASE = f"waraps/unit/{AgentConfig.DOMAIN}/{AgentConfig.SIM_REAL}/{args.name}"

        my_agent = Agent()

        # Send agent data 'rate' times per second
        rate: float = 1.0 / my_agent.logic.rate
        while True:
            my_agent.send_heartbeat()
            my_agent.send_sensor_info()
            my_agent.send_position()

            time.sleep(rate)

    except Exception:
        print(traceback.format_exc())


if __name__ == "__main__":
    main()
