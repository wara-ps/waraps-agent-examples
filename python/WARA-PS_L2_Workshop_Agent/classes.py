from dataclasses import dataclass
import paho.mqtt.client as paho_mqtt
from data.config import AgentConfig, MqttConfig, GpsConfig


@dataclass
class GpsClient:
    """Current sensor information"""
    def __init__(self):
        self.lat: float = GpsConfig.LATITUDE
        self.lon: float = GpsConfig.LONGITUDE
        self.alt: float = GpsConfig.ALTITUDE
        self.heading: float = GpsConfig.DUMMY_HEADING
        self.course: float = GpsConfig.DUMMY_COURSE
        self.speed: float = 0.0


@dataclass
class Logic:
    """Agent configuration"""
    def __init__(self):

        # Agent variables
        self.name: str = AgentConfig.NAME
        self.type: str = AgentConfig.TYPE
        self.description: str = AgentConfig.DESCRIPTION
        self.domain: str = AgentConfig.DOMAIN
        self.sim_real: str = AgentConfig.SIM_REAL
        self.level: list[str] = AgentConfig.LEVEL
        self.rate: float = AgentConfig.UPDATE_RATE
        self.uuid: str = AgentConfig.UUID
        self.tasks_available = AgentConfig.TASKS_AVAILABLE

        # Task variables
        self.task_running: bool = False
        self.task_pause_flag: bool = False
        self.task_running_uuid: str = ""
        self.task_paused: bool = False
        self.task_target: tuple = ()
        self.path: list = []

        print(f"Agent {self.name} Created")


@dataclass
class MqttClient:
    """Mqtt connection information"""
    def __init__(self):
        self.client: paho_mqtt.Client = paho_mqtt.Client()
        self.client.user_data_set("waraps")
        self.base_topic: str = MqttConfig.WARAPS_TOPIC_BASE
        self.listen_topic: str = MqttConfig.WARAPS_LISTEN_TOPIC
        self.user: str = MqttConfig.WARAPS_USERNAME
        self.password: str = MqttConfig.WARAPS_PASSWORD
        self.broker: str = MqttConfig.WARAPS_BROKER
        self.port: int = MqttConfig.WARAPS_PORT
        self.tls_connection: bool = MqttConfig.WARAPS_TLS_CONNECTION
