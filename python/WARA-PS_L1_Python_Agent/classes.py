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

        print(f"Agent {self.name} Created")


@dataclass
class MqttClient:
    """Mqtt connection information"""
    def __init__(self):
        self.client: paho_mqtt.Client = paho_mqtt.Client()
        self.client.user_data_set("waraps")
        self.base_topic: str = MqttConfig.WARAPS_TOPIC_BASE
        self.user: str = MqttConfig.WARAPS_USERNAME
        self.password: str = MqttConfig.WARAPS_PASSWORD
        self.broker: str = MqttConfig.WARAPS_BROKER
        self.port: int = MqttConfig.WARAPS_PORT
        self.tls_connection: bool = MqttConfig.WARAPS_TLS_CONNECTION
