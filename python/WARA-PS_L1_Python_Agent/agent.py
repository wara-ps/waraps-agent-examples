import json
import ssl
import time
import traceback
from classes import Logic, MqttClient, GpsClient


class Agent():
    def __init__(self) -> None:
        self.gps = GpsClient()
        self.logic = Logic()

        # MQTT setup
        self.mqtt_client = MqttClient()
        self.mqtt_client.client.on_connect = self.on_connect
        self.mqtt_client.client.on_disconnect = self.on_disconnect
        self.connect(self.mqtt_client)

    def connect(self, client):
        """Connect to the broker using the mqtt client"""
        if client.tls_connection:
            client.client.username_pw_set(client.user, client.password)
            client.client.tls_set(cert_reqs=ssl.CERT_NONE)
            client.client.tls_insecure_set(True)
        try:
            client.client.connect(client.broker, client.port, 60)
            client.client.loop_start()
        except Exception as exc:
            print(f"Failed to connect to broker {client.broker}:{client.port}")
            print(exc)
            exit()

    def publish(self, topic, msg):
        """Publish the message (msg) on the given topic"""
        self.mqtt_client.client.publish(topic, msg)

    def disconnect(self, client):
        """Disconnect the client from the broker"""
        client.client.disconnect()
        client.client.loop_stop()

    def on_connect(self, client, userdata, flags, rc):
        """Callback triggered when the client connects to the broker"""
        try:
            if rc == 0:
                print(f"Connected to MQTT Broker: {self.mqtt_client.broker}:{self.mqtt_client.port}")
            else:
                print(f"Error to connect : {rc}")
        except Exception:
            print(traceback.format_exc())

    def on_disconnect(self, client, userdata, rc):
        """Is triggered when the client gets disconnected from the broker"""
        print(f"Client Got Disconnected from the broker {userdata} with code {rc}")
        if rc == 5:
            print("No (or Wrong) Credentials, Edit in '.env'")

    def send_heartbeat(self):
        """Publish the heartbeat information of the agent to its topic"""
        json_msg = {
            "name": self.logic.name,
            "agent-type": self.logic.type,
            "agent-description": self.logic.description,
            "agent-uuid": self.logic.uuid,
            "levels": self.logic.level,
            "rate": self.logic.rate,
            "stamp": time.time(),
            "type": "HeartBeat"
        }
        str_msg = json.dumps(json_msg)
        self.publish(
            f"{self.mqtt_client.base_topic}/heartbeat", str_msg)

    def send_sensor_info(self):
        """Publish the sensor information of the agent to its topic"""
        json_msg = {
            "name": self.logic.name,
            "rate": self.logic.rate,
            "sensor-data-provided": [
                "position",
            ],
            "stamp": time.time(),
            "type": "SensorInfo"
        }
        str_msg = json.dumps(json_msg)
        self.publish(
            f"{self.mqtt_client.base_topic}/sensor_info", str_msg)

    def send_position(self):
        """Publish the position of the agent to its topic"""
        json_msg = {
            "latitude": self.gps.lat,
            "longitude": self.gps.lon,
            "altitude": self.gps.alt,
            "type": "GeoPoint"
        }
        str_msg = json.dumps(json_msg)
        self.publish(
            f"{self.mqtt_client.base_topic}/sensor/position", str_msg)
