from .config import TriggerAIConfig
from dataclasses import dataclass
import paho.mqtt.client as paho_mqtt

@dataclass
class TriggerAIMqttClient():
    def __init__(self):
        self.client: paho_mqtt = paho_mqtt.Client()
        self.client.user_data_set("waraps")
        self.user: str = TriggerAIConfig.WARAPS_USERNAME
        self.password: str = TriggerAIConfig.WARAPS_PASSWORD
        self.broker: str = TriggerAIConfig.WARAPS_BROKER
        self.port: int = TriggerAIConfig.WARAPS_PORT
        self.tls_connection: bool = TriggerAIConfig.WARAPS_TLS_CONNECTION
        self.push_videostream_listen_basetopic: str = TriggerAIConfig.PUSH_VIDEOSTREAM_LISTEN_BASETOPIC
        self.push_videostream_topic: str = TriggerAIConfig.PUSH_VIDEOSTREAM_TOPIC
        self.start_ai_topic: str = TriggerAIConfig.START_AI_TOPIC

# @dataclass
# class TriggerAILogic():
#     def __init__(self):
#         # Agent variables
#         self.name: str = AgentConfig.NAME
#         self.type: str = AgentConfig.TYPE
#         self.description: str = AgentConfig.DESCRIPTION
#         self.domain: str = AgentConfig.DOMAIN
#         self.sim_real: str = AgentConfig.SIM_REAL
#         self.level: str = AgentConfig.LEVEL
#         self.rate: float = AgentConfig.UPDATE_RATE
#         self.uuid: list = AgentConfig.UUID
#         self.tasks_available = AgentConfig.TASKS_AVAILABLE

#         # Task variables
#         self.task_running: bool = False
#         self.task_start_time: float = None
#         self.task_pause_flag: bool = False
#         self.task_running_uuid: str = ""
#         self.task_paused: bool = False
#         self.task_target: tuple = None
#         self.path: list = []

#         print(f"Agent {self.name} Created")        
