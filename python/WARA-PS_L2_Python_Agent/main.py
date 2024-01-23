import argparse
import time
import traceback
from agent import Agent
from data.config import AgentConfig, MqttConfig


def main():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-n", "--name", help="Give the agents a name")
        parser.add_argument("-u", "--units", help="Input how many agents to create")
        args = parser.parse_args()

        if args.name:
            AgentConfig.NAME = args.name
            agent_topic: str = f"waraps/unit/{AgentConfig.DOMAIN}/{AgentConfig.SIM_REAL}/{AgentConfig.NAME}"
            MqttConfig.WARAPS_TOPIC_BASE = agent_topic
            MqttConfig.WARAPS_LISTEN_TOPIC = f"{agent_topic}/exec/command"

        my_agent = Agent()

        # Listen for incoming messages and send agent data 'rate' times per second
        rate: float = 1.0 / my_agent.logic.rate
        while True:
            my_agent.check_task()
            my_agent.send_heartbeat()
            my_agent.send_sensor_info()
            my_agent.send_position()
            my_agent.send_speed()
            my_agent.send_course()
            my_agent.send_heading()
            my_agent.send_direct_execution_info()

            time.sleep(rate)

    except Exception:
        print(traceback.format_exc())


if __name__ == "__main__":
    main()
