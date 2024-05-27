/*
 * MQTT messages definitions
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

                 "\"sensor-data-provided\":[" +                            // Declaration of agent sensors
                      "\"position\","         +
                      "\"waypoints\","        +
                      "\"connectivity\","     +
                      "\"info\""              +
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

void sendConnectivity() {                    // Sensor RSSI for AccessPoint
  //String rssi = String(WiFi.RSSI()); // Change to signed integer or float
  int rssi = WiFi.RSSI();
  String json = "{\"type\": \"ap\", \"ip\": " + ip.toString() + ", \"rssi\": " + rssi + "}";

  // debugPrintln(d_always, "RSSI - " + rssi);
  pubAgent("sensor/connectivity", json);
}


// Sensor/L2 Messages -------------------------------------------------------------------

void sendDirectExecutionInfo(float rate) {   // Declaration of supported agent commands/tasks
  

  String json = "{\"name\": \""                 + a_name         + "\"," +  // Name/id of agent
                 "\"rate\": "                   + String(rate,1) + "," +    // Update rate in herts for the message

                 "\"tasks-available\":[{"       +                         // Declaration of agent tasks
                      "\"name\":\"search-area\","   +
                      "\"signals\":[]"          +
                      "}],"                     +
                 
                 "\"stamp\": "                  + timestamp      + "," +    // Epoch time
                 "\"type\": \""                 + "SensorInfo"   + "\"}";   // Type of message
  
  pubAgent("direct_execution_info", json);
}

void sendWaypoints() {                       // Sensor intention/curr wp
  String json;
  if (com_curr.state == s_running) {
    json  = "{\"waypoints\": " + jsonWaypoints() + "}";
  } else {
    json  = "{\"waypoints\": []}";
  }

  pubAgent("sensor/waypoints", json);

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


void sub(String top) {                        // Subscribe to MQTT topic. Wraps the original subscribe function with the use of strings instead of char arrays.
  mqtt_client.subscribe((char*) top.c_str());
}

void subAgent(String sub_top) {               // Subscribe to MQTT sub topic. 
  String top = a_topic_base  + "/" + a_domain + "/" + a_sim_real + "/" + a_name + "/" + sub_top;
  sub(top);
}


// Command ACK/NACK ---------------------------------------------------------------------------------------------------

void sendCommandACK(String c_UUID_old, String t_UUID) {                             // Command is verified and task can be executed
  sendResponse("running", "", c_UUID_old, t_UUID);
}

void sendCommandNACK(String nack_reason, String c_UUID_old, String t_UUID) {        // Command is received but task can not be executed
  sendResponse("failed", nack_reason, c_UUID_old, t_UUID);
  sendFeedback(nack_reason,           c_UUID_old, t_UUID);
}


// Task feedback ------------------------------------------------------------------------------------------------------

void sendTaskStarted(String c_UUID_old, String t_UUID) {                            // Task is started
  sendFeedback("started", c_UUID_old, t_UUID);
}

void sendTaskPlanning(String c_UUID_old, String t_UUID) {                           // Task is planned
  sendFeedback("planning", c_UUID_old, t_UUID);
}

void sendTaskRunning(String c_UUID_old, String t_UUID) {                            // Task is executing
  sendFeedback("running", c_UUID_old, t_UUID);
}

void sendTaskFinnished(String c_UUID_old, String t_UUID) {                          // Task is finnished succesfully
  sendResponse("finnished", "", c_UUID_old, t_UUID);
  sendFeedback("finnished",     c_UUID_old, t_UUID);
}

void sendTaskFailed(String fail_reason, String c_UUID_old, String t_UUID) {         // Task has failed
  sendResponse("failed", fail_reason, c_UUID_old, t_UUID);
  sendFeedback(fail_reason,           c_UUID_old, t_UUID);
}


// Command/Task/Signal helpers ----------------------------------------------------------------------------------------

void sendResponse(String txt, String txt_fail, String c_UUID_old, String t_UUID) {  // Response message - Command ACK [running] or NACK [failed]

  // Get unique UUID for the response message
  String cUUID = get_UUID();

  String json = "{\"agent-uuid\": \""  + a_UUID     + "\"," +   // The agents static UUID
                 "\"com-uuid\": \""    + cUUID      + "\"," +   // New generated UUID for each command/signal
                 "\"response\": \""    + txt        + "\"," +   // Put response text information here
                 "\"fail-reason\": \"" + txt_fail   + "\"," +   // Put error information about command/signal fail here
                 "\"response-to\": \"" + c_UUID_old + "\"," +   // Command UUID from "C2"
                 "\"task-uuid\": \""   + t_UUID     + "\"}";    // Original task UUID from "C2"

  if (txt_fail && txt_fail.length() > 0) {
    debugPrintln(d_status, "Send Response - " + txt + ", fail reason:" + txt_fail);
  } else {
    debugPrintln(d_status, "Send Response - " + txt);
  }
  
  pubAgent("exec/response", json);
}

void sendFeedback(String txt, String c_UUID_old, String t_UUID) {                   // Feedback message - Continuous feedback from task execution.

  // Get unique UUID for the feedback message
  String cUUID = get_UUID();

  String json = "{\"agent-uuid\": \""  + a_UUID     + "\"," +   // The agents static UUID
                 "\"com-uuid\": \""    + cUUID      + "\"," +   // New generated UUID for each command/signal
                 "\"status\": \""      + txt        + "\"," +   // Put status text information here
                 "\"response-to\": \"" + c_UUID_old + "\"," +   // Command UUID from "C2"
                 "\"task-uuid\": \""   + t_UUID     + "\"}";    // Original task UUID from "C2"

  debugPrintln(d_status, "Send Feedback - " + txt);

  pubAgent("exec/feedback", json);
}


// General information/debug ------------------------------------------------------------
void sendGeneralInfo(String source, String txt) {

  String json = "{\"source\": \"" + source + "\"," +  // Source of information
                 "\"info\": \""   + txt    + "\"}";   // Information text

  debugPrintln(d_status, "Send General Info - Source: " + source + ", Info: " + txt);

  pubAgent("sensor/info", json);
}
