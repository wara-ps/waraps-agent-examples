from data.config import AgentConfig
import os

#@dataclass(frozen=True)
class GazeboMqttConfig:
    ###Default Settings for MQTT###
    
    # ----------------------------------------------GAZEBO BROKER-------------------------------------------------------
    GAZEBO_TOPIC_BASE: str      = f"{AgentConfig.NAME}"
    GAZEBO_BROKER: str     = os.getenv('GAZEBO_BROKER')
    GAZEBO_PORT: int       = int(os.getenv('GAZEBO_PORT'))
    GAZEBO_TLS_CONNECTION: bool = bool(os.getenv('GAZEBO_TLS_CONNECTION', 'False') == 'TRUE') # Sets anything but 'TRUE' to false
    GAZEBO_USERNAME: str   = os.getenv('GAZEBO_USERNAME')
    GAZEBO_PASSWORD: str   = os.getenv('GAZEBO_PASSWORD')