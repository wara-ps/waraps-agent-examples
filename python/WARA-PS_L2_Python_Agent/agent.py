import json
import uuid
import ssl
import time
import math
import traceback
from haversine import haversine, inverse_haversine
from classes import Logic, MqttClient, GpsClient
from data.config import GpsConfig


class Agent():
    def __init__(self) -> None:
        self.gps = GpsClient()
        self.logic = Logic()

        # MQTT setup
        self.mqtt_client = MqttClient()
        self.mqtt_client.client.on_connect = self.on_connect
        self.mqtt_client.client.on_message = self.on_message
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
                self.mqtt_client.client.subscribe(self.mqtt_client.listen_topic)
                print(f"Subscribing to {self.mqtt_client.listen_topic}")
            else:
                print(f"Error to connect : {rc}")
        except Exception:
            print(traceback.format_exc())

    def on_message(self, client, userdata, msg):
        """Is triggered when a message is published on topics agent subscribes to"""
        try:
            msg_str = msg.payload.decode("utf-8")
            msg_json = json.loads(msg_str)

            if msg_json["command"] == "start-task":
                print("RECEIVED COMMAND 'start-task'")

                task_uuid = msg_json["task-uuid"]
                task = msg_json["task"]
                com_uuid = msg_json["com-uuid"]

                msg_res_json = {
                    "agent-uuid": self.logic.uuid,
                    "com-uuid": str(uuid.uuid4()),
                    "fail-reason": "",
                    "response": "",
                    "response-to": com_uuid,
                    "task-uuid": task_uuid
                }

                msg_feed_json = {
                    "agent-uuid": self.logic.uuid,
                    "com-uuid": str(uuid.uuid4()),
                    "status": "",
                    "task-uuid": task_uuid
                }

                if self.is_task_supported(task) and not self.logic.task_running:
                    if task["name"] == "move-to":
                        self.logic.task_running = True
                        self.logic.task_running_uuid = task_uuid
                        msg_res_json["response"] = "running"
                        msg_res_json["fail-reason"] = ""
                        msg_feed_json["status"] = "running"

                        lat = task["params"]["waypoint"]["latitude"]
                        lon = task["params"]["waypoint"]["longitude"]
                        alt = task["params"]["waypoint"]["altitude"]

                        self.logic.task_target = (lat, lon, alt)

                        self.initialize_speed(task["params"]["speed"])

                    elif task["name"] == "go-home":
                        self.logic.task_running = True
                        self.logic.task_running_uuid = task_uuid
                        msg_res_json["response"] = "running"
                        msg_res_json["fail-reason"] = ""
                        msg_feed_json["status"] = "running"

                        lat = GpsConfig.LATITUDE
                        lon = GpsConfig.LONGITUDE
                        alt = GpsConfig.ALTITUDE

                        self.logic.task_target = (lat, lon, alt)

                        self.initialize_speed(task["params"]["speed"])

                    elif task["name"] == "move-path":
                        self.logic.task_running = True
                        self.logic.task_running_uuid = task_uuid
                        msg_res_json["response"] = "running"
                        msg_res_json["fail-reason"] = ""
                        msg_feed_json["status"] = "running"

                        for point in task["params"]["waypoints"]:
                            self.logic.path.append(point)

                        lat = self.logic.path[0]["latitude"]
                        lon = self.logic.path[0]["longitude"]
                        alt = self.logic.path[0]["altitude"]

                        self.logic.path.pop(0)
                        self.logic.task_target = (lat, lon, alt)

                        self.initialize_speed(task["params"]["speed"])

                    elif task["name"] == "search-area":
                        self.logic.task_running = True
                        self.logic.task_running_uuid = task_uuid
                        msg_res_json["response"] = "running"
                        msg_res_json["fail-reason"] = ""
                        msg_feed_json["status"] = "running"

                        area_points = []
                        for point in task["params"]["area"]:
                            area_points.append(point)

                        self.plan_area_search(area_points)

                        self.initialize_speed(task["params"]["speed"])

                else:
                    if self.logic.task_running:  # Task running
                        msg_res_json["fail-reason"] = "A task is already running"
                    else:  # Task not supported
                        msg_res_json["fail-reason"] = "Task is not supported"
                    msg_res_json["response"] = "failed"
                    msg_feed_json["status"] = "failed"

                msg_res_str = json.dumps(msg_res_json)
                msg_feed_str = json.dumps(msg_feed_json)
                exec_topic: str = f"{self.mqtt_client.base_topic}/exec"
                self.mqtt_client.client.publish(f"{exec_topic}/response", msg_res_str)
                self.mqtt_client.client.publish(f"{exec_topic}/feedback", msg_feed_str)
                print(f"SENT RESPONSE! : {msg_res_str}")
                print(f"SENT FEEDBACK! : {msg_feed_str}")

            # Command that affects running task
            elif msg_json["command"] == "signal-task":
                print("RECEIVED COMMAND 'signal-task'")
                signal = msg_json["signal"]
                signal_task_uuid = msg_json["task-uuid"]
                com_uuid = msg_json["com-uuid"]

                msg_res_json = {
                    "com-uuid": str(uuid.uuid4()),
                    "response": "",
                    "response-to": com_uuid,
                    "task-uuid": self.logic.task_running_uuid
                }
                msg_feed_json = {
                    "agent-uuid": self.logic.uuid,
                    "com-uuid": str(uuid.uuid4()),
                    "status": "",
                    "task-uuid": self.logic.task_running_uuid
                }

                # Task signals
                if self.logic.task_running_uuid == signal_task_uuid:
                    if signal == "$abort":
                        msg_feed_json["status"] = "aborted"
                        self.reinstate_agent_variables()

                    elif signal == "$enough":
                        msg_feed_json["status"] = "enough"
                        self.reinstate_agent_variables()

                    elif signal == "$pause":
                        msg_feed_json["status"] = "paused"
                        self.logic.task_pause_flag = True

                    elif signal == "$continue":
                        msg_feed_json["status"] = "running"
                        self.logic.task_pause_flag = False

                    msg_res_json["response"] = "ok"
                else:
                    msg_res_json["fail-reason"] = "Invalid task-uuid"
                    msg_res_json["response"] = "failed"
                    msg_feed_json["status"] = "failed"

                msg_res_str = json.dumps(msg_res_json)
                msg_feed_str = json.dumps(msg_feed_json)
                exec_topic: str = f"{self.mqtt_client.base_topic}/exec"
                self.mqtt_client.client.publish(f"{exec_topic}/response", msg_res_str)
                self.mqtt_client.client.publish(f"{exec_topic}/feedback", msg_feed_str)
                print(f"SENT RESPONSE! : {msg_res_str}")
                print(f"SENT FEEDBACK! : {msg_feed_str}")

        except Exception:
            print(traceback.format_exc())

    def on_disconnect(self, client, userdata, rc):
        """Is triggered when the client gets disconnected from the broker"""
        print(f"Client Got Disconnected from the broker {userdata} with code {rc}")
        if rc == 5:
            print("No (or Wrong) Credentials, Edit in '.env'")

    def initialize_speed(self, speed: str) -> None:
        """Set current speed of agent according to string"""
        if speed == "slow":
            self.gps.speed = GpsConfig.SPEED_SLOW
        elif speed == "standard":
            self.gps.speed = GpsConfig.SPEED_STANDARD
        elif speed == "fast":
            self.gps.speed = GpsConfig.SPEED_FAST
        else:
            self.gps.speed = GpsConfig.SPEED_STANDARD

    def reinstate_agent_variables(self):
        """Reset agent's task variables"""
        self.logic.task_running_uuid = ""
        self.logic.task_running = False
        self.logic.task_pause_flag = False
        self.logic.task_target = ()
        self.logic.path = []

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
                "speed",
                "course",
                "heading",
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

    def send_speed(self):
        """Publish the speed of the agent to its topic"""
        speed = self.gps.speed
        self.publish(f"{self.mqtt_client.base_topic}/sensor/speed", speed)

    def send_course(self):
        """Publish the course of the agent to its topic"""
        course = self.gps.course
        self.publish(f"{self.mqtt_client.base_topic}/sensor/course", course)

    def send_heading(self):
        """Publish the heading of the agent to its topic"""
        heading = self.gps.heading
        self.publish(
            f"{self.mqtt_client.base_topic}/sensor/heading", heading)

    def send_direct_execution_info(self):
        """Publish the direct execution information of the agent to its topic"""

        json_msg = {
            "type": "DirectExecutionInfo",
            "name": self.logic.name,
            "rate": self.logic.rate,
            "stamp": time.time(),
            "tasks-available": self.logic.tasks_available
        }
        str_msg = json.dumps(json_msg)
        self.publish(f"{self.mqtt_client.base_topic}/direct_execution_info", str_msg)

    def set_speed(self, speed: float) -> None:
        """Set current speed according to float (m/s)"""
        self.gps.speed = speed

    def set_heading(self, heading: float) -> None:
        """Set the current heading"""
        self.gps.heading = heading

    def set_course(self, course: float) -> None:
        """Set the current course"""
        self.gps.course = course

    def move_to_target(self, current: tuple, target: tuple) -> None:
        """Move agent to the target position in Latitude, Longitude and Altitude"""
        *current_no_alt, _ = current
        *target_no_alt, _ = target

        # Check distance to target in kilometers
        distance = haversine(current_no_alt, target_no_alt)
        if distance <= 0.01:
            print("Reached target....", end=" ")
            self.gps.lat, self.gps.lon = target_no_alt
            if not self.logic.path:  # no path
                self.logic.task_running = False
                print("Task Complete")
            else:  # there is a path
                lat = self.logic.path[0]["latitude"]
                lon = self.logic.path[0]["longitude"]
                alt = self.logic.path[0]["altitude"]
                self.logic.task_target = (lat, lon, alt)
                self.logic.path.pop(0)
                print("Moving to next target")
            return

        bearing = self.bearing(current_no_alt, target_no_alt)
        self.set_heading(math.degrees(bearing))
        self.set_course(math.degrees(bearing))
        speed_km_per_second = self.gps.speed / 1000  # meters/second/1000 = km/second
        speed = speed_km_per_second / self.logic.rate
        new_location = inverse_haversine(current_no_alt, speed, bearing)

        # Change position of the Agent
        self.gps.lat, self.gps.lon = new_location

        # Code for altitude
        speed_m_per_rate = self.gps.speed / self.logic.rate
        distance_height = self.logic.task_target[2] - self.gps.alt

        if abs(distance_height) > self.gps.speed:
            if distance_height < self.gps.speed:
                self.descend(speed_m_per_rate)
            elif distance_height > self.gps.speed:
                self.ascend(speed_m_per_rate)
        else:
            self.gps.alt = self.logic.task_target[2]

    def ascend(self, speed: float) -> None:
        self.gps.alt += speed

    def descend(self, speed: float) -> None:
        self.gps.alt -= speed

    def plan_area_search(self, area_points):
        """Plan a zig-zag path for the agent to move over an area"""
        # Make use of maximum and minimum GeoPoints to "cover" the whole area
        lat_max = 0.0
        lat_min = 90.0
        lon_max = 0.0
        lon_min = 180.0
        alt_max = -10000.0
        alt_min = 10000.0

        for geopoint in area_points:
            lat_max = max(lat_max, geopoint["latitude"])
            lat_min = min(lat_min, geopoint["latitude"])
            lon_max = max(lon_max, geopoint["longitude"])
            lon_min = min(lon_min, geopoint["longitude"])
            alt_max = max(alt_max, geopoint["altitude"])
            alt_min = min(alt_min, geopoint["altitude"])

        area_divisions = 5
        lon_diff = (lon_max-lon_min)/area_divisions

        # Create travel points to create a zigzag path that cover the whole area
        for c in range(area_divisions+1):
            cut_lon = lon_min + (c * lon_diff)

            geopoint_A = {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0}
            geopoint_B = {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0}

            if (c % 2) == 0:
                geopoint_A["latitude"] = lat_min
                geopoint_B["latitude"] = lat_max
            else:
                geopoint_A["latitude"] = lat_max
                geopoint_B["latitude"] = lat_min

            geopoint_A["longitude"] = cut_lon
            geopoint_B["longitude"] = cut_lon
            geopoint_A["altitude"] = alt_min
            geopoint_B["altitude"] = alt_min

            self.logic.path.append(geopoint_A)
            self.logic.path.append(geopoint_B)

        # Set first point as first task target
        self.logic.task_target = (self.logic.path[0]["latitude"], self.logic.path[0]["longitude"], self.logic.path[0]["altitude"])
        self.logic.path.pop(0)

    def is_task_supported(self, task: dict) -> bool:
        """Check if the task is supported by the agent"""
        name: str = task["name"]
        task_supported: bool = False
        for ava_task in self.logic.tasks_available:
            if name == ava_task["name"]:
                task_supported = True
                break
        return task_supported

    def check_task(self) -> None:
        """Check if there is a task to perform and send feedback if it's finished"""
        if self.logic.task_running and not self.logic.task_pause_flag:
            self.move_to_target((self.gps.lat, self.gps.lon, self.gps.alt), self.logic.task_target)

            # Was task completed?
            if not self.logic.task_running:
                json_msg = {
                    "agent-uuid": self.logic.uuid,
                    "com-uuid": str(uuid.uuid4()),
                    "status": "finished",
                    "task-uuid": self.logic.task_running_uuid
                }
                self.logic.task_running_uuid = ""

                str_msg = json.dumps(json_msg)
                self.publish(f'{self.mqtt_client.base_topic}/exec/feedback', str_msg)

    def bearing(self, current: list, target: list) -> float:
        """Calculate bearing between two points"""
        startLat = math.radians(current[0])
        startLng = math.radians(current[1])
        destLat = math.radians(target[0])
        destLng = math.radians(target[1])
        y = math.sin(destLng - startLng) * math.cos(destLat)
        x = math.cos(startLat) * math.sin(destLat) - math.sin(startLat) * math.cos(destLat) * math.cos(destLng - startLng)
        brng = math.atan2(y, x)
        return brng
