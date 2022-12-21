from classes import Logic, MqttClient, GpsClient, GpsConfig
import json
import ssl
import time
import traceback

from haversine import haversine, inverse_haversine


class Agent():
    def __init__(self) -> None:

        ###GPS SETUP###
        self.gps = GpsClient()

        ###Agent LOGIC SETUP###
        self.logic = Logic()

        ##MQTT SETUP###
        self.mqtt_client = MqttClient()
        self.mqtt_client.client.on_connect = self.on_connect
        self.mqtt_client.client.on_disconnect = self.on_disconnect
        self.connect(self.mqtt_client)

       
    ########################################
    ########################################
    #################MQTT###################
    ########################################
    ########################################

    def connect(self, client):
        if client.tls_connection:
            client.client.username_pw_set(client.user, client.password)
            client.client.tls_set(cert_reqs=ssl.CERT_NONE)
            client.client.tls_insecure_set(True)

        client.client.connect(client.broker, client.port, 60)
        client.client.loop_start()

    def publish(self, topic, msg):
        result = self.mqtt_client.client.publish(topic, msg) 

    def disconnect(self, client):
        client.client.disconnect()
        client.client.loop_stop()

    # Callback function for PAHO
    # def on_connect(self, client, userdata, flags, rc):
    def on_connect(self, client, userdata, flags, rc):
        if(userdata == "waraps"):
            try:
                if rc == 0:
                    print(f"Connected to MQTT Broker: {self.mqtt_client.broker}:{self.mqtt_client.port}")
                else:
                    print(f"Error to connect : {rc}")
            except Exception as exc:
                print(traceback.format_exc())

    # Callback function for PAHO
    def on_disconnect(self, client, userdata, rc):
        print(f"Client Got Disconnected from the broker {userdata} with code {rc}")
        if rc == 5:
            print("No (or Wrong) Credentials, Edit in '.env'")

    def send_heartbeat(self):
        json_msg = {
            "name": self.logic.name,
            # "operator": "op1",
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
        json_msg = {
            "latitude": self.gps.lat,
            "longitude": self.gps.lon,
            "altitude": self.gps.alt,
            "type": "GeoPoint"
        }
        str_msg = json.dumps(json_msg)
        self.publish(
            f"{self.mqtt_client.base_topic}/sensor/position", str_msg)


    def send_direct_execution_info(self):
        json_msg = {
            "type": "DirectExecutionInfo",
            "name": self.logic.name,
            "rate": self.logic.rate,
            "stamp": time.time(),
            "tasks-available": self.logic.tasks_available
        }
        str_msg = json.dumps(json_msg)
        self.publish(f"{self.mqtt_client.base_topic}/direct_execution_info", str_msg)
