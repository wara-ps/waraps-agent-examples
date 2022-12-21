# from .config import TriggerAIConfig
from .classes import TriggerAIMqttClient
from .config import TriggerAIConfig
from data.config import AgentConfig
import json
import os
import ssl
import traceback
import paho
from dataclasses import dataclass

@dataclass
class TriggerAI:
    def __init__(self) -> None:

        ###CHECK BOOLS FOR STREAM AND AI###
        self.stream_pushed: bool = False #Is used to know when stream is pushed
        self.start_ai_pushed: bool = False #Is used to knot when ai is supposed to start

        self.agent_name: str = AgentConfig.NAME

        ###MQTT CONFIG###
        self.client = TriggerAIMqttClient()
        self.client.client.on_connect = self.on_connect
        self.client.client.on_disconnect = self.on_disconnect
        # self.client.push_videostream_topic = TriggerAIConfig.PUSH_VIDEOSTREAM_TOPIC
        # self.client.start_ai_topic = TriggerAIConfig.START_AI_TOPIC
        self.client.client.message_callback_add(f"{self.client.push_videostream_listen_basetopic}/pushes", self.handle_videostream_push)
        self.client.client.message_callback_add(f"{self.client.push_videostream_listen_basetopic}/ai", self.handle_ai_start)
        self.connect(self.client)

        # self.client.client.subscribe(f"{self.client.listen_topic}")

    def connect(self, client):
        if client.tls_connection:
            client.client.username_pw_set(client.user, client.password)
            client.client.tls_set(cert_reqs=ssl.CERT_NONE)
            client.client.tls_insecure_set(True)

        client.client.connect(client.broker, client.port, 60)
        client.client.loop_start()
    
    def on_connect(self, client, userdata, flags, rc):
        if(userdata == "waraps"):
            try:
                if rc == 0:
                    print(f"Connected to MQTT Broker: {self.client.broker}:{self.client.port}")
                    self.client.client.subscribe(f"{self.client.push_videostream_listen_basetopic}/#")
                    print(f"Subcribing to {self.client.push_videostream_listen_basetopic}")
                else:
                    print(f"Error to connect : {rc}")
            except Exception as exc:
                print(traceback.format_exc())

    def on_disconnect(self, client, userdata, rc):
        print(f"Client Got Disconnected from the broker {userdata} with code {rc}")
        if rc == 5:
            print("No (or Wrong) Credentials, Edit in 'secrets.py'")

    def publish(self, topic, msg):
        result = self.client.client.publish(topic, msg) 

    def publish_videostream_push(self, agent_name):
        if os.getenv('ENABLE_AI') != 'TRUE':
            return
        self.publish(self.client.push_videostream_topic, agent_name)
        print("Published videostream push")

    def handle_videostream_push(self, client, userdata, msg):
        if os.getenv('ENABLE_AI') != 'TRUE':
            return
        if not self.start_ai_pushed:
            try:
                msg_str = msg.payload.decode("utf-8")
                pushed_streams = json.loads(msg_str)

                if self.agent_name in pushed_streams:

                    payload = {
                        "toggle": "start",
                        "source": f"rtmp://rtmp.waraps.org/live/{self.agent_name}",
                        "agent": self.agent_name
                    }

                    payload_msg = json.dumps(payload)
                    self.publish(self.client.start_ai_topic, payload_msg)
                    print("Published start ai")
                    self.start_ai_pushed = True

            except Exception as e:
                print(traceback.format_exc())

    def handle_ai_start(self, client, userdata, msg):
        if os.getenv('ENABLE_AI') != 'TRUE':
            return
        if self.start_ai_pushed:
            try:
                msg_str = msg.payload.decode("utf-8")
                started_ais = json.loads(msg_str)
                if self.agent_name in started_ais:
                    print("AI on video stream started")

            except Exception as e:
                print(traceback.format_exc())

    def stop_ai_on_stream(self):
        if os.getenv('ENABLE_AI') != 'TRUE':
            return
        payload = {
            "toggle": "stop",
            "source": f"rtmp://rtmp.waraps.org/live/{self.agent_name}",
            "agent": self.agent_name
        }

        payload_msg = json.dumps(payload)
        self.client.client.publish(self.client.start_ai_topic, payload_msg)
        print("Published stop AI")