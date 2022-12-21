from dataclasses import dataclass
from data.config import AgentConfig
from pyproj import Proj
import json
import traceback
import math
import os
from .classes import GazeboMqttClient
import ssl

@dataclass
class Gazebo:
    def __init__(self, lat=57.7605573519, lon=16.6827607783, alt=29.8,  location=os.getenv('GAZEBO_LOCATION')):
        self.camera_heading: float = 1.0 # dummy value
        self.ori = [1.0, 0.0, 0.0, 0.0] #facing east


        # MQTT CONFIG
        self.client = GazeboMqttClient()

        ### Gazebo Mqtt Setup ###
        self.client.client.on_connect = self.on_connect
        self.client.client.on_disconnect = self.on_disconnect
        self.connect()

        # LOCATION CONFIG
        self.locations = {}
        self.locations["granso"] = (57.7605573519, 16.6827607783, 29.8, 27.9)
        self.locations["motala"] = (58.4948444444, 15.1022444444, 132.9, 29.95)
        self.locations["petterstedt"] = (58.323751, 15.406307, 128.0, 29.8)
        self.locations["terra"] = (58.3954055556, 15.5723222222, 104.4, 29.4)

        if location and location in self.locations:
            print("INITIALIZING FROM LOCATION:", location)
            self.lat0 = self.locations[location][0]
            self.lon0 = self.locations[location][1]
            self.alt0 = self.locations[location][2]
            self.sea_level = self.locations[location][3]
        else:
            self.lat0 = lat
            self.lon0 = lon
            self.alt0 = alt

# TODO: CHECK IF NEEDED
        print("ORIGIN:", lon, lat, alt)
        self.zone = self.utm_zone(self.lon0, self.lat0)
        print("UTM ZONE:", self.zone)
        self.proj = Proj(proj='utm', zone=self.zone, ellps='WGS84', preserve_units=False)        
        (x, y) = self.proj(self.lon0, self.lat0)
        print("UTM ORIGIN:", x, y)
        self.utm_x = x
        self.utm_y = y


    def connect(self):
        if self.client.tls_connection:
            self.client.client.username_pw_set(self.client.user, self.client.password)
            self.client.client.tls_set(cert_reqs=ssl.CERT_NONE)
            self.client.client.tls_insecure_set(True)

        self.client.client.connect(self.client.broker, self.client.port, 60)
        self.client.client.loop_start()

    def publish(self, topic, msg):
        result = self.client.client.publish(topic, msg) 

    def disconnect(self):
        self.client.disconnect()
        self.client.loop_stop()

    def on_connect(self, client, userdata, flags, rc):
            try:
                if rc == 0:
                    print(f"Connected to Gazebo Broker: {self.client.broker}:{self.client.port}")
                else:
                    print(f"Error to connect : {rc}")
            except Exception as exc:
                print(traceback.format_exc())

    def on_disconnect(self, client, userdata, rc):
        print(f"Client Got Disconnected from the broker {userdata} with code {rc}")
        if rc == 5:
            print("No (or Wrong) Credentials, Edit in 'secrets.py'")

    def calc_camera_feedback(self, dest_pos_lat, dest_pos_lon, curr_pos_lat, curr_pos_lon):
        if os.getenv('ENABLE_GAZEBO') != 'TRUE':
            return
        conv_dest_x, conv_dest_y, conv_dest_z = self.wgs84_to_world(dest_pos_lon,dest_pos_lat)
        conv_curr_x, conv_curr_y, conv_curr_z = self.wgs84_to_world(curr_pos_lon, curr_pos_lat)
        conv_course = (conv_dest_x - conv_curr_x, conv_dest_y - conv_curr_y)

        # x = 0
        if(conv_course[0] == 0):
            if(conv_course[1] > 0):
                self.set_camera_heading(math.pi/2)
            else:
                self.set_camera_heading(-math.pi/2)        

        # y = 0  
        elif(conv_course[1] == 0):
            if(conv_course[0] > 0):
                self.set_camera_heading(0)
            else:
                self.set_camera_heading(math.pi)

        else:
            a = math.acos(abs(conv_course[0]) / abs(math.sqrt(math.pow(conv_course[0],2) + math.pow(conv_course[1],2))))

            # x- y+
            if(conv_course[0] < 0 and conv_course[1] > 0):
                rad = math.pi - a
                self.set_camera_heading(rad)

            # y+ x+
            elif(conv_course[0] > 0 and conv_course[1] > 0):
                self.set_camera_heading(a)

            # x- y-
            elif(conv_course[0] < 0 and conv_course[1] < 0):
                rad = a - math.pi  
                self.set_camera_heading(rad)

            # x+ y-
            elif(conv_course[0] > 0 and conv_course[1] < 0):
                rad = -a
                self.set_camera_heading(rad)              

        self.ori = self.quaternion_from_euler(0, math.pi/6, self.camera_heading)

    def set_camera_heading(self, camera_heading: float) -> None:

        self.camera_heading = camera_heading

    def utm_zone(self, lon, lat):
        res = (int)((lon + 180.0) / 6.0) + 1
        if lat >= 56.0 and lat < 64.0 and lon >= 3.0 and lon < 12.0:
            res = 32
        # Special zones for Svalbard
        if lat >= 72.0 and lat < 84.0:
            if lon >= 0.0  and lon <  9.0:
                res = 31
            elif lon >= 9.0  and lon < 21.0:
                res = 33
            elif lon >= 21.0 and lon < 33.0:
                res = 35
            elif lon >= 33.0 and lon < 42.0:
                res = 37
        return res

    def world_to_wgs84(self, x, y, z=0.0):
        try:
            ux = self.utm_x + x
            uy = self.utm_y + y
            (lon, lat) = self.proj(ux, uy, inverse=True)
            alt = self.alt0 + z
            return (lon, lat, alt)
        except Exception as exc:
            print("EXCEPTION world_to_wgs84", type(exc))
            print(exc)

    def wgs84_to_world(self, lon, lat, alt=None):
        try:
            if alt == None:
                alt = self.alt0
            #print("wgs84_to_world:", lon, lat, alt)
            (x, y) = self.proj(lon, lat)
            return (x - self.utm_x, y - self.utm_y, alt - self.alt0)
        except Exception as exc:
            print("EXCEPTION wgs84_to_world", type(exc))
            print(exc)

         # Code for agent to display camera feed from gazebo
    def send_camera_feed(self, agent_name, curr_lon, curr_lat, curr_alt):
        if os.getenv('ENABLE_GAZEBO') != 'TRUE':
            return 
        # Converting drones current coordinates to the coordinate system of Gazebo 
        conv_coords = self.wgs84_to_world(curr_lon, curr_lat, self.sea_level + curr_alt)

        dict_msg_lrs = {
                "link_states": [
                    {
                        "link_name": agent_name +"::camera0_link",
                        "pose":{
                            # Camera orientation.
                            "orientation":{
                                "w":self.ori[0],
                                "x":self.ori[1],
                                "y":self.ori[2],
                                "z":self.ori[3]
                            },
                            "position":{
                                "x":0.0,
                                "y":0.0,
                                "z":-1 # 1m below drone
                            }
                        },
                        "reference_frame": agent_name +"::base_link",
                        "twist":{
                            "angular":{
                                "x":0.0,
                                "y":0.0,
                                "z":0.0
                            },
                            "linear":{
                                "x":0.0,
                                "y":0.0,
                                "z":0.0
                            }
                        }
                    }
                ],
                "model_state":{
                    "model_name": agent_name,
                    "pose":{
                        # Dummy values
                        "orientation":{
                            "w":1.0,
                            "x":0.0,
                            "y":0.0,
                            "z":0.0
                        },
                        "position":{
                            "x":conv_coords[0],
                            "y":conv_coords[1],
                            "z":conv_coords[2]
                        }
                    },
                    "reference_frame":"world",
                    "twist":{
                        "angular":{
                            "x":0.0,
                            "y":0.0,
                            "z":0.0
                        },
                        "linear":{
                            "x":0.0,
                            "y":0.0,
                            "z":0.0
                        }
                    }
                }
        }
        

        json_msg_lrs = json.dumps(dict_msg_lrs)
        self.publish(f"{self.client.base_topic}/gazebo/lrs_update" , json_msg_lrs)
        self.publish(f"{self.client.base_topic}/model_type", '"m100"')
        self.publish(f"{self.client.base_topic}/model_have_camera", "true")


    def quaternion_from_euler(self, roll, pitch, yaw):

        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)

        return [qw, qx, qy, qz]