from .config import GazeboMqttConfig
import paho.mqtt.client as paho_mqtt
from dataclasses import dataclass

@dataclass
class GazeboMqttClient:
    def __init__(self) -> None:
        self.client: paho_mqtt = paho_mqtt.Client()
        self.client.user_data_set("gazebo")
        self.base_topic: str = GazeboMqttConfig.GAZEBO_TOPIC_BASE
        self.user: str = GazeboMqttConfig.GAZEBO_USERNAME
        self.password: str = GazeboMqttConfig.GAZEBO_PASSWORD
        self.broker: str = GazeboMqttConfig.GAZEBO_BROKER
        self.port: int = GazeboMqttConfig.GAZEBO_PORT
        self.tls_connection: bool = GazeboMqttConfig.GAZEBO_TLS_CONNECTION
