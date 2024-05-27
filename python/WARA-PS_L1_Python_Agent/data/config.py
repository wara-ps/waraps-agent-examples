import uuid
import random
import string
import math
import os


class AgentConfig:
    @staticmethod
    def id_generator(size=4, chars=string.ascii_lowercase + string.digits):
        return "".join(random.choice(chars) for _ in range(size))

    unique_str = id_generator()

    NAME = f"pa_{unique_str}"  # Base name of agent. OBS! will be overwritten if -n is used in .yml file
    TYPE = "air"  # Type of agent.
    DESCRIPTION = "L1_agent_example"  # Description of agent.
    DOMAIN = "air"  # The domain for the agent (lowercase): [ground, surface, subsurface, air].
    SIM_REAL = "simulation"  # Is the agent simulated or real: [simulation, real].
    LEVEL = ["sensor"]  # Define agent level.
    UPDATE_RATE = 0.4  # Rate
    UUID = str(uuid.uuid4())  # Unique UUID
    TASKS_AVAILABLE = []


class GpsConfig:
    @staticmethod
    def random_start_pos():  # Agent is spawned a a random position around a 100m radius from the specified point at Gränsö.
        original_y = 57.76115009154693  # Hardcoded point at Gränsö
        original_x = 16.684087827200084
        r = 400 / 111300  # = 100 meters.
        y0 = original_y
        x0 = original_x
        u = random.uniform(0, 1)
        v = random.uniform(0, 1)
        w = r * math.sqrt(u)
        t = 2 * math.pi * v
        x = w * math.cos(t)
        y1 = w * math.sin(t)
        x1 = x / math.cos(y0)
        newY = y0 + y1
        newX = x0 + x1
        return (newY, newX)

    lat, lon = random_start_pos()
    LATITUDE: float = lat  # WGS84 latitude in decimal degrees + => north - => south.
    LONGITUDE: float = lon  # WGS84 longitude in decimal degrees + => east - => west.
    ALTITUDE: float = 29.8  # Ellipsoid altitude in decimal


class MqttConfig:
    """ Default MQTT settings """
    WARAPS_TOPIC_BASE: str = f"waraps/unit/{AgentConfig.DOMAIN}/{AgentConfig.SIM_REAL}/{AgentConfig.NAME}"
    WARAPS_BROKER: str = os.getenv("WARAPS_BROKER", default="Could not read .env file")
    WARAPS_PORT: int = int(os.getenv("WARAPS_PORT", default=0))
    WARAPS_TLS_CONNECTION: bool = bool(os.getenv("WARAPS_TLS_CONNECTION", "FALSE") == "TRUE")
    WARAPS_USERNAME: str = os.getenv("WARAPS_USERNAME", default="")
    WARAPS_PASSWORD: str = os.getenv("WARAPS_PASSWORD", default="")
