/*
 * L1 Agent MQTT messages definitions
 */

// Sensor/L1 Messages -------------------------------------------------------------------

void sendHeartbeat(float rate){              // Discovery service & declaration of supported agent level

  String levStr = "\"sensor\"";

  if (a_level > 1) {
    levStr += ", \"direct execution\"";
  }

  String json = "{\"name\": \""               + a_name          + "\"," +  // Name/id of agent
                 "\"agent-type\": \""         + a_type          + "\"," +  // Type of agent
                 "\"agent-description\": \""  + a_descr         + "\"," +  // Description of agent
                 "\"agent-uuid\": \""         + a_UUID          + "\"," +  // Unique UUID of agent

                 "\"levels\":["               + levStr          + "],"  +  // Declaration of agent integration levels
                 
                 "\"rate\": "                 + String(rate, 1) + "," +    // Update rate in herts for the message
                 "\"stamp\": "                + timestamp       + "," +    // Epoch time
                 "\"type\": \""               + "HeartBeat"     + "\"}";   // Type of message

  pubAgent("heartbeat", json);
}

void sendSensorInfo(float rate) {            // Declaration of supported agent sensors

  String json = "{\"name\": \""               + a_name          + "\"," +  // Name/id of agent
                 "\"rate\": "                 + String(rate, 1) + "," +    // Update rate in herts for the message

                 "\"sensor-data-provided\":[" +                         // Declaration of agent sensors
                      "\"position\","         +
                      "\"speed\","            +
                      "\"course\","           +
                      "\"heading\","          +
                      "\"camera_url\","       +
                      "\"connectivity\""      +
                 "],"                         +
                 
                 "\"stamp\": "                + timestamp        + "," +    // Epoch time
                 "\"type\": \""               + "SensorInfo"     + "\"}";   // Type of message

  pubAgent("sensor_info", json);
}

void sendPosition() {                        // Sensor position

  String json = "{\"latitude\": "  + String(a_pos_data.lat, 6) + "," +
                 "\"longitude\": " + String(a_pos_data.lon, 6) + "," +
                 "\"altitude\": "  + String(a_pos_data.alt, 2) + "," +
                 "\"rostype\": \"" + "GeoPoint" + "\"}";

  pubAgent("sensor/position", json);
}

void sendSpeed() {                           // Sensor speed
  pubAgent("sensor/speed", a_pos_data.spd);
}

void sendCourse() {                          // Sensor course
  pubAgent("sensor/course", a_pos_data.crs);
}

void sendHeading() {                         // Sensor heading
  pubAgent("sensor/heading", a_pos_data.hdg);
}

void sendCameraUrl() {                       // Sensor camera
  pubAgent("sensor/camera_url", a_cam_url);
}

void sendConnectivity() {                    // Sensor RSSI for AccessPoint
  //String rssi = String(WiFi.RSSI()); // Change to signed integer or float
  int rssi = WiFi.RSSI();
  String json = "{\"type\": \"ap\", \"ip\": " + ip.toString() + ", \"rssi\": " + rssi + "}";

  // debugPrintln(d_always, "RSSI - " + rssi);
  pubAgent("sensor/connectivity", json);
}


// General information/debug ------------------------------------------------------------
void sendGeneralInfo(String source, String txt) {

  String json = "{\"source\": \"" + source + "\"," +  // Source of information
                 "\"info\": \""   + txt    + "\"}";   // Information text

  debugPrintln(d_status, "Send General Info - Source: " + source + ", Info: " + txt);

  pubAgent("sensor/info", json);
}


// MQTT pub/sub helper functions ---------------------------------------

void pub(String top, String pl) {             // Publish to MQTT topic. Wraps the original publish function with the use of strings instead of char arrays.
  ledOn();
  mqtt_client.publish((char*) top.c_str(), (char*) pl.c_str());
  ledOff();

  delay(mqtt_msg_wait_ms);
}

void pub(String top, float pl) {              // Publish to MQTT topic. Wraps the original publish function with the use of strings and float instead of char arrays.
  String out = String(pl);

  pub(top, out);
}

void pubAgent(String sub_top, String pl) {    // Publish to agent MQTT sub topic.
  String top = a_topic_base  + "/" + a_domain + "/" + a_sim_real + "/" + a_name + "/" + sub_top;
  pub(top, pl);
}

void pubAgent(String sub_top, float pl) {     // Publish to agent MQTT sub topic.
  String top = a_topic_base  + "/" + a_domain + "/" + a_sim_real + "/" + a_name + "/" + sub_top;
  pub(top, pl);
}
