import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import ssl
from std_msgs import msg
import builtins
import paho.mqtt.client as mqtt
import json
import uuid
import time

class MqttPublishNode(Node):

    def __init__(self):
        super().__init__('get_position')

        #Declare parameters that are set in the config.yaml file
        self.declare_parameters(
            namespace='',
            parameters=[
                ('agent_name', rclpy.Parameter.Type.STRING),
                ('rate', rclpy.Parameter.Type.DOUBLE),
                ('username', rclpy.Parameter.Type.STRING),
                ('password', rclpy.Parameter.Type.STRING),
                ('host', rclpy.Parameter.Type.STRING),
                ('port', rclpy.Parameter.Type.INTEGER),
                ('base_topic', rclpy.Parameter.Type.STRING),
                ('agent_type', rclpy.Parameter.Type.STRING),
                ('levels', rclpy.Parameter.Type.STRING_ARRAY),
                ('tls', rclpy.Parameter.Type.BOOL)
                ]),

        #Declare dict that will be sent to mqtt broker
        self.position: dict = None

        #Agent parameters
        self.agent_name = self.get_parameter('agent_name').value
        self.agent_type = self.get_parameter('agent_type').value
        self.levels = self.get_parameter('levels').value
        self.agent_uuid = str(uuid.uuid4())
        self.rate = self.get_parameter('rate').value
        self.tls_connection: bool = self.get_parameter('tls').value
        self.sensor_data_provided = ["position", "speed", "heading", "course"]

        #Get MQTT relevant parameters and create client
        self.client = mqtt.Client()
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.base_topic = self.get_parameter('base_topic').value
        
        #Callback when client is connected to broker
        self.client.on_connect = self.on_connect

        #Connect client to broker with settings
        if self.tls_connection:
            self.client.username_pw_set(self.get_parameter('username').value, self.get_parameter('password').value)
            self.client.tls_set(cert_reqs=ssl.CERT_NONE)
            self.client.tls_insecure_set(True)
        self.client.connect(self.host, self.port, 60)
        self.client.loop_start()
        
        #Subscribe when L2 agent
        #self.client.subscribe(self.get_parameter('base_topic').value, qos=1) #to subscribe more than one topic add more subscribe lines
        
        #Create timer to publish to MQTT broker with set rate
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Subscribe to ros topic to get position values
        self.subscription = self.create_subscription(
            Point,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def on_connect(self, client, userdata, flags, rc):
        """Callback function for connecting to MQTT broker"""
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT Broker: {self.host}:{self.port}")

    def timer_callback(self):
        """Publish to MQTT every call"""

        #-------position--------
        self.get_logger().info('Publishing on mqtt: "%s"' % self.position)
        self.client.publish(f"{self.base_topic}{self.agent_name}/sensor/position", json.dumps(self.position))

        #-speed-heading-course--
        self.client.publish(f"{self.base_topic}{self.agent_name}/sensor/speed", 0.0)
        self.client.publish(f"{self.base_topic}{self.agent_name}/sensor/heading", 0.0)
        self.client.publish(f"{self.base_topic}{self.agent_name}/sensor/course", 0.0)

        #-------sensor-info----

        sensor_info = {
            "name": self.agent_name,
            "rate": self.rate,
            "sensor-data-provided": self.sensor_data_provided,
            "stamp": time.time(),
            "type": "SensorInfo"
        }
        self.client.publish(f"{self.base_topic}{self.agent_name}/sensor_info", json.dumps(sensor_info))

        #-------heartbeat------
        heartbeat = {
            "agent-type": self.agent_type,
            "agent-uuid": self.agent_uuid,
            "levels": self.levels,
            "name": self.agent_name,
            "rate": self.rate,
            "stamp": time.time(),
            "type": "HeartBeat"
        }
        self.client.publish(f"{self.base_topic}{self.agent_name}/heartbeat", json.dumps(heartbeat))

    def listener_callback(self, msg):
        """Updates position with msg on callback from another node"""
        self.position = {
            "longitude": msg.x,
            "latitude": msg.y,
            "altitude": msg.z
        }

def main(args=None):
    rclpy.init(args=args)

    node = MqttPublishNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()