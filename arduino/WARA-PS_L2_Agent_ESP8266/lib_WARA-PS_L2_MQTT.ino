/*
 * L2 Agent MQTT messages definitions
 */

// Sensor/L2 Messages -------------------------------------------------------------------

void sendDirectExecutionInfo(float rate) {   // Declaration of supported agent commands/tasks
  

  String json = "{\"name\": \""                 + a_name         + "\"," +  // Name/id of agent
                 "\"rate\": "                   + String(rate,1) + "," +    // Update rate in herts for the message

                 "\"tasks-available\":[{"       +                         // Declaration of agent tasks
                      "\"name\":\"move-to\","   +
                      "\"signals\":["           +
                          "\"abort\","          +
                          "\"enough\","         +
                          "\"pause\","          +
                          "\"continue\"]"       +
                          "},{"                 +
                      "\"name\":\"move-path\"," +
                      "\"signals\":["           +
                          "\"abort\","          +
                          "\"enough\","         +
                          "\"pause\","          +
                          "\"continue\"]"       +
                          "},{"                 +
                      "\"name\":\"go-home\","   +
                      "\"signals\":["           +
                          "\"abort\","          +
                          "\"enough\","         +
                          "\"pause\","          +
                          "\"continue\"]"       +
                      "}],"                     +
                 
                 "\"stamp\": "                  + timestamp      + "," +    // Epoch time
                 "\"type\": \""                 + "SensorInfo"   + "\"}";   // Type of message
  
  pubAgent("direct_execution_info", json);
}

void sendWaypoints() {                       // Sensor intention/curr wp
  String json;
  if (com_curr.state == s_running && planned_waypoints.curr_i != -1) {
   json  = "{\"waypoints\": [{\"latitude\": "  + String(a_pos_data.lat, 6) + "," +
                             "\"longitude\": " + String(a_pos_data.lon, 6) + "," +
                             "\"altitude\": "  + String(a_pos_data.alt, 2) + "," +
                             "\"rostype\": \"" + "GeoPoint" + "\"}," + 

                            "{\"latitude\": "  + String(planned_waypoints.ps[planned_waypoints.curr_i].lat, 6) + "," +
                             "\"longitude\": " + String(planned_waypoints.ps[planned_waypoints.curr_i].lon, 6) + "," +
                             "\"altitude\": "  + String(planned_waypoints.ps[planned_waypoints.curr_i].alt, 2) + "," +
                             "\"rostype\": \"" + "GeoPoint" + "\"}]}";
  } else {
    json  = "{\"waypoints\": []}";
  }

  pubAgent("sensor/waypoints", json);

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

void sendTaskPaused(String c_UUID_old, String t_UUID) {                             // Task is paused 
  sendFeedback("paused", c_UUID_old, t_UUID);
}

void sendTaskFinnished(String c_UUID_old, String t_UUID) {                          // Task is finnished succesfully
  sendResponse("finnished", "", c_UUID_old, t_UUID);
  sendFeedback("finnished",     c_UUID_old, t_UUID);
}

void sendTaskFailed(String fail_reason, String c_UUID_old, String t_UUID) {         // Task has failed
  sendResponse("failed", fail_reason, c_UUID_old, t_UUID);
  sendFeedback(fail_reason,           c_UUID_old, t_UUID);
}


// Signal ACK/NACK ----------------------------------------------------------------------------------------------------

void sendSignalAbortACK(String c_UUID_old, String t_UUID) {                         // Signal has been received and task is cancelled unsuccessfully
  sendResponse("OK"    , "", c_UUID_old, t_UUID);
  sendFeedback("failed",     c_UUID_old, t_UUID);
}

void sendSignalEnoughACK(String c_UUID_old, String t_UUID) {                        // Signal has been received and task is cancelled successfully
  sendResponse("OK"    , "", c_UUID_old, t_UUID);
}

void sendSignalPauseACK(String c_UUID_old, String t_UUID) {                         // Signal has been received and task has been paused
  sendResponse("OK"    , "", c_UUID_old, t_UUID);
}

void sendSignalContinueACK(String c_UUID_old, String t_UUID) {                      // Signal has been received and task is continuing from paused state
  sendResponse("OK"    , "", c_UUID_old, t_UUID);
}

void sendSignalNACK(String nack_reason, String c_UUID_old, String t_UUID) {         // Signal has been received but is not supported/recognized
  sendResponse("failed", nack_reason, c_UUID_old, t_UUID);
  sendFeedback("failed",              c_UUID_old, t_UUID);
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

void sendPong(String c_UUID_old, String t_UUID) {                                   // Handshake ping/pong for L3 and L4 agents

  // Get unique UUID for the response message
  String cUUID = get_UUID();

  String json = "{\"agent-uuid\": \""  + a_UUID     + "\"," +   // The agents static UUID
                 "\"com-uuid\": \""    + cUUID      + "\"," +   // New generated UUID for each command/signal
                 "\"response\": \""    + "pong"     + "\"," +   // Put response text information here
                 "\"response-to\": \"" + c_UUID_old + "\"}";    // Original task UUID from "C2"


  debugPrintln(d_status, "Send Pong");

  pubAgent("exec/response", json);
    
}


// MQTT sub helper functions (pub helper functions are defined i WARA-PS_MQTT_L1.ino) ---------------------------------------

void sub(String top) {                        // Subscribe to MQTT topic. Wraps the original subscribe function with the use of strings instead of char arrays.
  mqtt_client.subscribe((char*) top.c_str());
}

void subAgent(String sub_top) {               // Subscribe to MQTT sub topic. 
  String top = a_topic_base  + "/" + a_domain + "/" + a_sim_real + "/" + a_name + "/" + sub_top;
  sub(top);
}
