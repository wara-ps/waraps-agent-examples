from classes import Logic, MqttClient, GpsClient
from data.config import GpsConfig
import json
import uuid
import ssl
import time
import math
import traceback
from modules.gazebo.gazebo import Gazebo
from modules.trigger_ai.trigger_ai import TriggerAI

from haversine import haversine, inverse_haversine


class Agent():
    def __init__(self) -> None:

        ###GPS SETUP###
        self.gps = GpsClient()

        ###Agent LOGIC SETUP###
        self.logic = Logic()

        ###Gazebo module class###
        self.gazebo = Gazebo()

        ###Start AI on stream module class###
        self.trigger_ai = TriggerAI()

        ##MQTT SETUP###
        self.mqtt_client = MqttClient()
        self.mqtt_client.client.on_connect = self.on_connect
        self.mqtt_client.client.on_message = self.on_message
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
                    self.mqtt_client.client.subscribe(f"{self.mqtt_client.listen_topic}")
                    print(f"Subcribing to {self.mqtt_client.listen_topic}")
                else:
                    print(f"Error to connect : {rc}")
            except Exception as exc:
                print(traceback.format_exc())

    # Callback function for PAHO
    def on_message(self, client, userdata, msg):
        if(userdata == "waraps"):
            try:
                msg_str = msg.payload.decode("utf-8")
                msg_json = json.loads(msg_str)
                print(msg_json)

                if msg_json["command"] == "ping":
                    print("RECIVED COMMAND 'PING'")
                    msg_res_json = {
                        "com-uuid": msg_json["com-uuid"],
                        "response": "pong",
                        "response-to": msg_json["com-uuid"]
                    }
                    msg_res_str = json.dumps(msg_res_json)
                    self.mqtt_client.client.publish(f'{self.mqtt_client.base_topic}/exec/response', msg_res_str)
                    print(f"SENT RESPONSE! : {msg_res_str}")

                elif msg_json["command"] == "start-task":
                    print("RECIVED COMMAND 'start-task'")

                    task_uuid = msg_json["task-uuid"]
                    task = msg_json["task"]
                    com_uuid = msg_json["com-uuid"]

                    msg_res_json = {
                        "agent-uuid": self.logic.uuid,
                        "com-uuid": com_uuid,
                        "fail-reason": "",
                        "response": "",
                        "response-to": com_uuid,
                        "task-uuid": task_uuid
                    }

                    msg_feed_json = {
                        "agent-uuid": self.logic.uuid,
                        "com-uuid": com_uuid,
                        "status": "",
                        "task-uuid": task_uuid
                    }

                    if self.is_task_supported(task) and not self.logic.task_running:

                        if task["name"] == "move-to":
                            self.task_start_time = time.time()
                            self.logic.task_running = True
                            self.logic.task_running_uuid = task_uuid
                            msg_res_json["response"] = "running"
                            msg_res_json["fail-reason"] = ""
                            msg_feed_json["status"] = "running"

                            lat = task["params"]["waypoint"]["latitude"]
                            lon = task["params"]["waypoint"]["longitude"]
                            alt = task["params"]["waypoint"]["altitude"]
                            self.logic.task_target = (lat, lon, alt)
                            
                            self.initialize_speed(task)

                        if task["name"] == "move-path":
                            self.task_start_time = time.time()
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

                            self.initialize_speed(task)

                        if task["name"] == "go-home":
                            self.task_start_time = time.time()
                            self.logic.task_running = True
                            self.logic.task_running_uuid = task_uuid
                            msg_res_json["response"] = "running"
                            msg_res_json["fail-reason"] = ""
                            msg_feed_json["status"] = "running"

                            lat = GpsConfig.LATITUDE
                            lon = GpsConfig.LONGITUDE
                            alt = GpsConfig.ALTITUDE

                            self.logic.task_target = (lat, lon, alt)

                            self.initialize_speed(task)

                        if task["name"] == "search-area":
                            self.task_start_time = time.time()
                            self.logic.task_running = True
                            self.logic.task_running_uuid = task_uuid
                            msg_res_json["response"] = "running"
                            msg_res_json["fail-reason"] = ""
                            msg_feed_json["status"] = "running"

                            area_points = []
                            for point in task["params"]["area"]:
                                area_points.append(point)

                            self.plan_area_search(area_points)

                            self.initialize_speed(task)

                            #Push video stream
                            self.trigger_ai.publish_videostream_push(self.logic.name)

                    else:
                        if self.logic.task_running:  # Task running
                            msg_res_json["fail-reason"] = "A task is already running"
                        else:  # Task not supported
                            msg_res_json["fail-reason"] = "Task is not supported"
                        msg_res_json["response"] = "failed"
                        msg_feed_json["status"] = "failed"

                    msg_res_str = json.dumps(msg_res_json)
                    msg_feed_str = json.dumps(msg_feed_json)
                    self.mqtt_client.client.publish(f'{self.mqtt_client.base_topic}/exec/response', msg_res_str)
                    self.mqtt_client.client.publish(f'{self.mqtt_client.base_topic}/exec/feedback', msg_feed_str)
                    print(f"SENT RESPONSE! : {msg_res_str}")
                    print(f"SENT FEEDBACK! : {msg_feed_str}")

                elif msg_json["command"] == "signal-task":
                    print("RECIVED COMMAND 'signal-task'")
                    signal = msg_json["signal"]
                    signal_task_uuid = msg_json["task-uuid"]
                    com_uuid = msg_json["com-uuid"]

                    msg_res_json = {
                        "com-uuid": com_uuid,
                        "response": "",
                        "response-to": com_uuid,
                        "task-uuid": self.logic.task_running_uuid
                    }
                    msg_feed_json = {
                        "agent-uuid": self.logic.uuid,
                        "com-uuid": com_uuid,
                        "status": "",
                        "task-uuid": self.logic.task_running_uuid
                    }

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
                        msg_res_json["response"] = "failed"
                        msg_feed_json["status"] = "failed"

                    msg_res_str = json.dumps(msg_res_json)
                    msg_feed_str = json.dumps(msg_feed_json)
                    self.mqtt_client.client.publish(f'{self.mqtt_client.base_topic}/exec/response', msg_res_str)
                    self.mqtt_client.client.publish(f'{self.mqtt_client.base_topic}/exec/feedback', msg_feed_str)
                    print(f"SENT RESPONSE! : {msg_res_str}")
                    print(f"SENT FEEDBACK! : {msg_feed_str}")

            except Exception as e:
                print(traceback.format_exc())

    # Callback function for PAHO
    def on_disconnect(self, client, userdata, rc):
        print(f"Client Got Disconnected from the broker {userdata} with code {rc}")
        if rc == 5:
            print("No (or Wrong) Credentials, Edit in 'secrets.py'")

    def send_videoserver_url(self):
        topic = f"{self.mqtt_client.base_topic}/sensor/videoserver_url"
        self.mqtt_client.client.publish(topic, f"{self.mqtt_client.base_videoserver_url}/{self.logic.name}")

    def initialize_speed(self, task):
        speed = task["params"]["speed"]
        if speed == "slow":
            self.gps.speed = GpsConfig.SPEED_SLOW
        elif speed == "standard":
            self.gps.speed = GpsConfig.SPEED_STANDARD
        elif speed == "fast":
            self.gps.speed = GpsConfig.SPEED_FAST
        else:
            self.gps.speed = GpsConfig.SPEED_STANDARD
    
    def reinstate_agent_variables(self):

        #Task variables
        self.logic.task_running = False
        self.task_target= None
        self.path= []

        #Stop the AI (AI module)
        self.trigger_ai.stop_ai_on_stream()

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
        speed = self.gps.speed
        self.publish(f"{self.mqtt_client.base_topic}/sensor/speed", speed)

    def send_course(self):
        course = self.gps.course
        self.publish(f"{self.mqtt_client.base_topic}/sensor/course", course)

    def send_heading(self):
        heading = self.gps.heading
        self.publish(
            f"{self.mqtt_client.base_topic}/sensor/heading", heading)

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

    # Not used for this agent
    # def send_tst_execution_info(self):
    #     json_msg = {
    #         "name": self.logic.name,
    #         "rate": self.logic.rate,
    #         "stamp": time.time(),
    #         "type": "TstExecutionInfo"
    #     }
    #     str_msg = json.dumps(json_msg)
    #     self.publish(f"{self.mqtt_client.base_topic}/tst_execution_info", str_msg)

    ########################################
    ########################################
    #################LOGIC##################
    ########################################
    ########################################

    def ascend(self, speed: float) -> None:
        self.gps.alt += speed

    def decend(self, speed: float) -> None:
        self.gps.alt -= speed

    def alter_altitude(self, speed: float, altitude: float) -> None:
        """Changes the agents altitude with a check to not go to far"""
        if self.gps.alt > altitude:
            if (self.gps.alt - speed) < altitude:
                self.gps.alt = altitude
            else:
                self.decend(speed)
        elif self.gps.alt < altitude:
            if (self.gps.alt + speed) > altitude:
                self.gps.alt = altitude
            else:
                self.ascend(speed)

    def set_speed(self, speed: float) -> None:
        self.gps.speed = speed

    def set_heading(self, heading: float) -> None:
        self.gps.heading = heading

    def set_course(self, course: float) -> None:
        self.gps.course = course

    def plan_area_search(self, area_points):
        """Plans a zig-zag path for the agent to move over an area"""
        # Make use of maximum and minimum GeoPoints to "cover" the whole area
        lat_max = 0.0
        lat_min = 90.0
        lon_max = 0.0
        lon_min = 180.0
        alt_max = -10000.0
        alt_min = 10000.0

        # The maximum travel points allowed the agent is allowed to travel in order to "cover" the area
        max_travel_points = 20

        for i in range(len(area_points)): 
            geopoint = area_points[i] 

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

            if (c%2) == 0:
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

    def move_to_target(self):
        """Moves to the taget position in Latitude, Longitude and Altitude"""

        current_no_alt = (self.gps.lat, self.gps.lon)
        *target_no_alt, t_alt = self.logic.task_target

        # *current_no_alt, _ = current
        # *target_no_alt, t_alt = target
        # alt_speed: float = 10.0
        # travel_distance: float = 0.001

        # Distance in Kilometer
        distance = haversine( current_no_alt, target_no_alt )

        #Speed in kilometers per second
        speed_km_per_second = self.gps.speed /1000 #meter/second /1000 = km/second
        speed = speed_km_per_second / self.logic.rate

        # Check if close to target
        if distance <= 0.01: #and self.gps.alt == t_alt:
            print(f"Reached target....", end=" ")
            self.gps.lat, self.gps.lon = target_no_alt
            if not self.check_path(): #no path
                self.logic.task_running = False
                print("Task Complete")
            else: #there is a path
                lat = self.logic.path[0]["latitude"]
                lon = self.logic.path[0]["longitude"]
                alt = self.logic.path[0]["altitude"]
                self.logic.task_target = (lat, lon, alt)
                self.logic.path.pop(0)
                print("Moving to next target")
            return

        #TODO this is a #fullösning since we don't use the altitude in the arena for now
        # display_alt_speed: float = t_alt - self.gps.alt
        # if display_alt_speed < 0: display_alt_speed *= -1 #If negative value, change it

        bearing = self.bearing(current_no_alt,target_no_alt)
        self.set_heading(math.degrees(bearing))
        self.set_course(math.degrees(bearing))
        new_location = inverse_haversine(current_no_alt,speed,bearing)
        new_distance = haversine(new_location,target_no_alt)

        # åk inte för långt #fullösning
        if new_distance >= distance:
            new_location = inverse_haversine(current_no_alt,speed,bearing)

        self.gps.lat, self.gps.lon = new_location

        #Gazebo camera calculation (Gazebo module)
        self.gazebo.calc_camera_feedback(self.logic.task_target[0],self.logic.task_target[1],self.gps.lat, self.gps.lon)

    def is_task_supported(self,task: json) -> bool:
        """Checks if the task is supported by the agent"""
        name: str = task["name"]
        task_supported: bool = False
        for ava_task in self.logic.tasks_available:
            if name == ava_task["name"]:
                task_supported = True
                break
        return task_supported
    
    def check_task(self) -> None:
        """Checks if there is a task to preform and send feeback if its finished"""  
        if self.logic.task_running:
            if self.logic.task_pause_flag: #Task is paused
                pass
            else: #Task is running
                self.move_to_target()

                if not self.logic.task_running:
                    json_msg = {
                        "agent-uuid": self.logic.uuid,
                        "com-uuid": str(uuid.uuid4()), #TODO skickar rätt com-uuid
                        "status": "finished",
                        "task-uuid": self.logic.task_running_uuid
                    }
                    self.task_running_uuid = ""

                    str_msg = json.dumps(json_msg)
                    self.publish(f'{self.mqtt_client.base_topic}/exec/feedback',str_msg)

    def bearing(self, current: tuple, target: tuple) -> float:
        """Calculates bearing between two points"""
        startLat = math.radians(current[0])
        startLng = math.radians(current[1])
        destLat = math.radians(target[0])
        destLng = math.radians(target[1])
        y = math.sin(destLng - startLng) * math.cos(destLat)
        x = math.cos(startLat) * math.sin(destLat) - math.sin(startLat) * math.cos(destLat) * math.cos(destLng - startLng)
        brng = math.atan2(y, x)
        return brng

    def check_path(self):
        """Returns the path"""
        return self.logic.path
