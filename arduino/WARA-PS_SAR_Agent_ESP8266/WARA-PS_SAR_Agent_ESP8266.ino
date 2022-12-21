/* ================================================================================================================================
  Name:         WARA-PS_SAR_Agent_ESP8266(Mesh of: WARA-PS_L2_Agent_ESP8266.ino & WARA-PS_Services_ESP8266.ino)
  API version:  0.7 (WARA-PS Core System API Specification)
  IDE:          Arduino IDE 1.8.15
  Target:       Tested on WeMos/Lolin D1 Mini but should work on all ESP8266/32/12 development boards 
  Developer:    Original files by Combitech AB (Michael Petterstedt). Modified into SAR agent by Combitech AB (Caroline Flodin) - Librariy examples have been used for inspiration

  DESCRIPTION ---------------------
  This is an example application of task agent integration (Level 2) with WARA-PS Core System. More specifically an agent capable of searching an area.
  OBS! The agent only moves as a matter of 10 steps, meaning we are unable to correctly display the speed an agent of the type actually has.
  The application demonstrates how to implement the WARA-PS Core System API Specification.

  The agent connects to the WARA-PS Core System, through the MQTT broker, report itself as an agent in the air domain. The 
  properties that are sent are:
  - position
  - speed
  - course
  - heading
  - camera_url
  - waypoints     // The planned path by the agent when a movement command has been received.-
  - info          // General info from the agent when receiving a command.
  If GPS is connected and the GPS_CONNECTED variable (in hw-setup.h file) is set to true the agent will 
  update the properties related to a GPS receiver.

  The L2 agent also implements listeners for simple commands from the WARA-PS Core System. These commands are:
  - move-to
  - move-path
  - search-area
  - go-home
  Commands can be aborted, paused/continued and finnished.

  The application can be verified in the WARA-PS Integration Test Tool (https://2021.nodered.waraps.org/ui) 
  where the agents meta data, position etc. can be reviewed aswell as sending supported commands.

  SETUP IDE ----------------------
  - Install Arduino IDE + Drivers https://www.arduino.cc/en/software
  - Install ESP core/boards       https://arduino.esp8266.com/Arduino/versions/2.0.0/doc/installing.html
  - Install Libraries
    - ESP8266WiFi                 Preinstalled by board manager
    - WiFiUdp                     Preinstalled by board manager
    - SoftwareSerial              Preinstalled by board manager
    - PubSubClient                [Menu - Tools - Manage Libraries] -> Search for PubSubClient (Nick O’Leary) -> Install
    - NTPClient                   [Menu - Tools - Manage Libraries] -> Search for NTPClient (Fabrice Weinberg) -> Install
    - jsonlib                     [Menu - Tools - Manage Libraries] -> Search for jsonlib (Justin Shaw) -> Install
    - MicroNMEA                   [Menu - Tools - Manage Libraries] -> Search for MicroNMEA (Steve Marple) -> Install
    - SimpleTimer                 https://playground.arduino.cc/Code/SimpleTimer/
    - Servo                       Preinstalled by board manager

  BUILD & DEPLOY APPLICATION -----
  - Start Arduino IDE and open ino-file WAPA-PS_Lx_Agent_ESP8266.ino.
    - IDE will contain main projects file as well as header files (see below) and helper ino-files.
    - Parameters that are capitalized are defined in separate header files.
  - Mandatory:
    - Define user connection parameters/secrets in the secrets.h file.
    - Choose ths used ESP-board in [Tools - Board - ESP8266 boards - Lolin (WeMos) D1 Mini] or the board of your choice.
    - Click the Verify/Compile button (or CTRL-R) and check build results, correct if necessary.
    - Connect the ESP8266 development board to computer.
    - Click the Upload button (or CTRL-U). The agent should flash its built in LED once per second when sending messages to MQTT broker.
  - Optional:
    - Setup agent parameters in the agent-setup.h file. The agent will work without touching this file.
    - Setup hardware properties in the hw-setup.h file. The agent will work without touching this file.

  DEBUG & VERIFICATION -----------
  - Debug by [Menu - Tools - Serial Monitor] (or CTRL-SHIFT-M). Ensure baudrate is 57600.
      (Progress and debug messages are sent to virtual serial port on USB. Add your own messages at choice).
  - Verify by MQTT Explorer (or your favourite MQTT tool)
  - Verify by WARA-PS Integration Test tool (https://2021.nodered.waraps.org/ui).

  TIP
  - Enable expand/compress of code blocks by [File - Preferences - Enable Code Folding]. 
      Fold/unfold by click at margin or right click in code [Folding - Collapse/Expand All Folds].
  - You can change the debug flag (d_threshold) to present more debug information if you want.
  - Check used libraries by [File - Preferences - Show verbose output during: compilation].

  NOTES --------------------------
  - SSL Certificate is not checked
  =================================================================================================================================== */

#include <ESP8266WiFi.h>            // WiFi connectivity
#include <WiFiUdp.h>                // UDP handler used by NTPClient
#include <SoftwareSerial.h>         // Enables GPS reading from (allmost) any GPIO
#include <PubSubClient.h>           // MQTT client for publish and subscribe to and from MQTT server
#include <NTPClient.h>              // NTPC client for setting actual time (GMT)
#include <jsonlib.h>                // JSON-parser for received MQTT messages
#include <MicroNMEA.h>              // For GPS parsing 
#include <SimpleTimer.h>            // Timer for sending MQTT messages
#include <Servo.h>                  // "PWM" Library for controlling servo

#include "secrets.h"                // Define secret parameters as passwords etc.
#include "agent-setup.h"            // Define name, home position etc.
#include "hw-setup.h"               // Settings for gps, servo etc.


// COMMUNICATION SETTINGS (WiFi & MQTT) - Defined in secrets.h ===========================
const char*   wifi_ssid1      = SECRET_WIFI_SSID1;      // SSID for Access point (Primary).
const char*   wifi_pass1      = SECRET_WIFI_PASS1;      // Password for access point (Primary).
const char*   wifi_ssid2      = SECRET_WIFI_SSID2;      // SSID for Access point (Secondary).
const char*   wifi_pass2      = SECRET_WIFI_PASS2;      // Password for access point (Secondary).

const char*   mqtt_server     = "broker.waraps.org";    // Host name or IP address for MQTT broker.
const int     mqtt_port       = 8883;                   // 1883, 8883(ssl).
const char*   mqtt_user       = SECRET_MQTT_USER;       // User name for MQTT broker.
const char*   mqtt_pass       = SECRET_MQTT_PASS;       // Password for MQTT broker.
String        client_name;                              // Unique client name => auto generated at setup.
char*         NTP_url         = "pool.ntp.org";         // Network Time Protocol Server url

// AGENT DECLARATION - Defined in agent-declaration.h ====================================
String        a_name;                                   // Name of agent, a unique identifier is added at setup.

const String  a_topic_base    = A_TOPIC_BASE;           // Base topic for agent, e.g. "waraps/unit".
const String  a_base_name     = A_BASE_NAME;            // Base name of agent.
const String  a_type          = A_TYPE;                 // Type of agent.
const String  a_descr         = A_DESCR;                // Description of agent.
const String  a_domain        = A_DOMAIN;               // The domain for the agent (lowercase): [ground, surface, subsurface, air].
const String  a_sim_real      = A_SIM_REAL;             // Is the agent simulated or real: [simulation, real].
const int     a_level         = A_LEVEL;                // Agent level.

const String  a_cam_url       = A_CAM_URL;              // Camera example source. Shall be a rtsp but the Core System can also handle rtmp streams (contact support).

// GPS CONFIGURATION - Defined in hw-setup.h ==============================================
const bool    gps_connected   = GPS_CONNECTED;
const int     gps_RX_pin      = GPS_RX_PIN;
const int     gps_TX_pin      = GPS_TX_PIN;
const int     gps_baud        = GPS_BAUD;

SoftwareSerial gpsSerial(gps_RX_pin, gps_TX_pin);       // Serial port on gps_RX_pin and gps_TX_pin. Hardware serial does not work, do not know why!?

// MicroNMEA library -----------
char          nmea_buffer[200];                         // Buffer for GPS parser.
MicroNMEA     nmea(nmea_buffer, sizeof(nmea_buffer));   // GPS/NMEA parser.

typedef struct PosData {
  float       lat             = 0.0;                    // WGS84 latitude       in decimal degrees + => north - => south.
  float       lon             = 0.0;                    // WGS84 longitude      in decimal degrees + => east  - => west.
  float       alt             = 0.0;                    // Ellipsoid altitude   in decimal meters.
  float       spd             = 0.0;                    // Speed over ground    in decimal meters/second.
  float       crs             = 0.0;                    // Course over ground   in decimal degrees, north up.
  float       hdg             = 0.0;                    // Heading              in decimal degrees, north up.
};

PosData       a_pos_data;                               // Storage for actual position/GPS Data.

// WGS84 GeoPoint for position and WP handling -------
typedef struct GeoPoint {
  float       lat             = 0.0;                    // WGS84 latitude       in decimal degrees + => north - => south.
  float       lon             = 0.0;                    // WGS84 longitude      in decimal degrees + => east  - => west.
  float       alt             = 0.0;                    // Ellipsoid altitude   in decimal meters.
};

// MQTT helpers --------------------
const    int            mqtt_msg_wait_ms  = 50;         // Milliseconds to wait for MQTT message to be sent (> 10 is recommended when using servo).
const    int            max_mqtt_bytes    = 4096;
String                  mqtt_string;
volatile unsigned int   no_of_mqtt_bytes = 0;           // Length of buffer.

// DEBUG PRINTS ============================================================================
const int     d_threshold     = 3;                      // Threshold level (>=) for output prints

const int     d_always        = 4;
const int     d_error         = 3;
const int     d_warning       = 2;
const int     d_status        = 1;
const int     d_test          = 0;

// MISC ====================================================================================
String        x_UUID          =                         // Default dummy UUID.
  "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeeeeeee";
String        a_UUID          = x_UUID;                 // Agent UUID. Generated once per start.

unsigned long start_millis    = 0;                      // Helper for measuring time during MQTT messaging.
unsigned long timestamp       = 0;                      // Epoch Time in seconds.
bool          task_exec_1Hz   = false;                  // Flag for executing commanded task in time event.
int           sim_tick        = 0;                      // Simulator variable
int           max_sim_ticks   = 10;                     // Simulator variable


// Start CUT FOR REMOVING L2+ DECLARATIONES ====================================================================================================================

// L2 COMMAND HANDLING =====================================================================

// State Definitions ----------------
const int     s_error         = -1;
const int     s_undefined     =  0;
const int     s_started       =  1;
const int     s_planning      =  2;
const int     s_running       =  3;
const int     s_paused        =  4;
const int     s_finnished     =  5;

const String  states[] = { // Corresponding to the above indexes except errors.
  "undefined", "started", "planning", "running", "paused", "finnished"
};

// Task Handling --------------------
typedef struct Command {
  String      com_type        = "";                     // Command type: [start-task, signal-task, pin, ...].
  String      sender          = "";                     // Commander name.
  String      task_name       = "";                     // Task name: [move-to, move-path, ...].
  String      speed           = "";                     // Commanded speed: [slow, standard, fast].
  String      c_UUID_old      = "";                     // Command UUID from commander, to identify the commander.
  String      t_UUID          = "";                     // Task UUID, to identify the task and subsequent signals.
  int         state           = s_undefined;            // Current state of the command, see State Definitions above.
  int         state_old       = s_undefined;            // Previous state of the command, see State Definitions above.
};

Command       com_curr;                                 // Current/Last command that was received.

// Command helpers (Input Data) ----------
const int     max_points      = 20;                     // Maximum points in a command. Must be declared.
const float   near_WP         = 5.0;                   // Approximity threshold in meters, trigger next WP in a path or task finnished.
bool          sim_new_WP      = false;                  // True when new Waypoint is initiated. Simulation help variable.

typedef struct Positions {
  GeoPoint      ps[max_points];                         // Array with geopoints holding the commanded point/path (1 - max_points). Must be declared.
  int           end_i  = -1;                            // Index of last/end point in points array.
  int           curr_i = -1;                            // Index of current point, e.g. a WP to navigate to.
};

Positions command_positions;
Positions planned_waypoints;

//Extra variables for SearchAreaPlanning
float areaWidthMeters = 0.0;
float areaLengthMeters = 0.0;
bool wideArea = false;
int currArrayLength = 4;

// Servo Declaration  - Defined in hw-setup.h ----------------
Servo         servo;                                    // Servo handler.
bool          servo_connected = SERVO_CONNECTED;
int           servo_pin       = SERVO_PIN;
int           servo_angle_max = SERVO_ANGLE_MAX;
int           servo_angle_min = SERVO_ANGLE_MIN;
bool          servo_invert    = SERVO_INVERT;

// END CUT FOR REMOVING L2+ DECLARATIONES ====================================================================================================================

// Clients & Settings -------------
WiFiClientSecure  wifi_client;
PubSubClient      mqtt_client(wifi_client);
WiFiUDP           wifi_udp_client;
NTPClient         time_client(wifi_udp_client, NTP_url);
SimpleTimer       timer;
IPAddress         ip;
ADC_MODE(ADC_VCC);


// TBD: SSL cerificate checks will be implemented in the future (see below, wifi_client.setInsecure())
// static const char *fingerprint PROGMEM = "44 14 9A 3F C3 E9 F1 F3 84 1A B4 9F B6 4D 19 8A B2 92 31 D6";

// MAIN SETUP ==========================================================

void setup() {
  // Enabling built in LED for simple status - Started
  pinMode(LED_BUILTIN, OUTPUT);

  // Initializing serial port for debugging and status reporting
  Serial.begin(57500);
  debugPrintln(d_always, "Setup - Started");
  ledStatus(1); // Show started by 1 flash on LED

  // Set a unique UUID for the agent.
  a_UUID = get_UUID();

  // Set "unique" identifier/name. Enables multiple users with the same example code.
  // The name is the same between after each start.
  a_name      = getUniqueName();
  client_name = a_name;

  // Placing agent at home.
  a_pos_data.lat = A_HOME_LAT;
  a_pos_data.lon = A_HOME_LON;
  a_pos_data.alt = A_HOME_ALT;

  // Connecting to WiFi access point
  setupWiFi();
  ledStatus(2);  // Show connected to WiFi by 2 flashes on LED

  // Synchronize time, start NTP Service
  setupNTP();
  ledStatus(3);  // Show connected to NTP server by 3 flashes on LEd

  // Connecting to MQTT broker
  setupMQTT();
  ledStatus(4);  // Show connected to broker by 4 flashes on LED

  // Setup GPS serial port
  if (gps_connected) {
    setupGPS();
  }

  // Setup servo pin and set servo to straight ahead
  if (servo_connected) {
    setupServo();
  }
  
  // Setting up event triggers
  setupTriggers();

  ledStatus(5);  // Show setup finnished by 5 flashes on LED

  delay(1000);   // Wait for 1 seconds to be able to see the final flashes.

}


// MAIN LOOP ===========================================================

void loop() {

  // NOTE: Most agent mqtt activities are executed by events that is initiated in the setup function

  // if MQTT client disconnects, of some reason, it tries to reconnect.
  if (!mqtt_client.connected()) {
    setupMQTT();
  }

  // Place main code here as: getting timestamp, parsing of gps, temperature, etc. by polling. GPS can of course be implemented as event driven serial port listener.
  // Main principle is to write to global volatile variables that is handled by time events.
  // Currently the example getters does not do anything.

  getAgentPositionSpeedCourseHeading();

  validator();    // Parse and validate commands/signals from MQTT broker + change state upon validated signals. 
  executor();     // Execute validated commands and hold the states and make state transitions.

  // Synchronize mqtt and time events
  mqtt_client.loop();
  timer.run();

}


// AGENT SENSORS/LISTENERS ===========================================================
// set global variables, e.g. from parsing of GPS etc.

void getAgentPositionSpeedCourseHeading() {

//  noInterrupts();

  if (gps_connected) {

    // Get GPS position from serial port
    if (gpsSerial.available()) {

      bool newPos = false;

      while (gpsSerial.available()) {
        char c = gpsSerial.read();
        newPos = nmea.process(c);
      }

      if(newPos && nmea.isValid()) {
        updateAgentPositionGPS();
        debugPrintln(d_always, "GPS - Position updated");
      }

    }

  // Simple simulation - Step through the path array of waypoints
  } else {
    
    // Run once a second when having a task running
    if (com_curr.state == s_running && task_exec_1Hz && planned_waypoints.curr_i > -1 && planned_waypoints.end_i  > -1) {
      if (planned_waypoints.curr_i <= planned_waypoints.end_i ) {
        debugPrintln(d_status, "Agent Position from simulator");

        float new_lat;
        float new_lon;
        float new_alt;
        
        printWaypoints();
        
        if (com_curr.state == s_started || sim_new_WP) {
          debugPrintln(d_test, "####### 1");
          sim_tick = 0;

          new_lat   = a_pos_data.lat;
          new_lon   = a_pos_data.lon;
          new_alt   = a_pos_data.alt;
          
          sim_new_WP = false;

        // Recalculate intermediate positions
        } else if (sim_tick < (max_sim_ticks-1) ) {
          debugPrintln(d_test, "####### 2");
          sim_tick++;
          new_lat   = a_pos_data.lat + (planned_waypoints.ps[planned_waypoints.curr_i].lat - a_pos_data.lat)/(10.0-sim_tick);
          new_lon   = a_pos_data.lon + (planned_waypoints.ps[planned_waypoints.curr_i].lon - a_pos_data.lon)/(10.0-sim_tick);
          new_alt   = a_pos_data.alt + (planned_waypoints.ps[planned_waypoints.curr_i].alt - a_pos_data.alt)/(10.0-sim_tick);
        }

        float crs = getBearing2CurrWP();
        float spd = getDistanceBetween(a_pos_data.lat, a_pos_data.lon, new_lat, new_lon); // Distance is in meters, sim_tick is 1 second thus meters/second.

        // Agent is at final waypoint
        updateAgentPosition(new_lat, new_lon, new_alt, spd, crs, crs);
      }
    }
  }

//  interrupts();

}


// AGENT MQTT COMMAND LISTENER/CALLBACK (L2 Agent) ===================================

void mqttListener(char* topic, byte* payload, unsigned int length) {

//  noInterrupts();

  // Write payload to volatile global variable
  payload[length]   = '\0';
  no_of_mqtt_bytes  = length;

  // BE AWARE THAT PUB & SUB USES THE SAME BUFFER. YOU MUST MAKE A TRUE COPY OF THE BUFFER.
  mqtt_string = String((char *)payload);

  debugPrintln(d_status, "MQTT Listener - Command/Signal received. Number of bytes: " + String(no_of_mqtt_bytes));

//  interrupts();
}


// AGENT MQTT COMMUNICATION ==========================================================

void send_1Hz() {   // Medium update rate
  debugPrint(d_always, ".");
  
  // Synchronize and store epoch time
  // Light weight implementation with one seconds resolution
  timestamp = getEpoch();

  float update_rate = 1.0; // Hz
  
  sendHeartbeat(update_rate);
  sendPosition();
  sendSpeed();
  sendCourse();
  sendHeading();
  sendWaypoints();

  task_exec_1Hz = true;
}

void send_0_2Hz() { // Low update rate
  debugPrintln(d_always, "|");

  float update_rate = 0.2; // Hz

  sendSensorInfo(update_rate);
  sendDirectExecutionInfo(update_rate);
  sendCameraUrl();
  sendConnectivity();

  int period_time = checkPeriod();
  if (period_time > (1000.0/update_rate) * 1.2) { // Trigger 20 % over limit
    debugPrintln(d_warning, "Main Loop - Period time for program execution is too long: " + String(period_time) + " milliseconds (> 5000). Check WiFi connectivity.");
  }
}

int checkPeriod() { // Reports the time between execution cyclies
  int time_spent = 0;

  // First time
  if (start_millis != 0) {
    time_spent = (millis() - start_millis);
  }
  start_millis = millis();
  return time_spent;
}


// COMMAND VALIDATORS/EXECUTORS ======================================================

void validator() {            // Parse & validate received command/signals. Commands are transfered as tasks for executor and signals are executed directly.

  // Check if new MQTT command has been received
  if(no_of_mqtt_bytes > 0) {

    debugPrintln(d_always, "Validator - Command/Signal received, no of bytes: " + String(no_of_mqtt_bytes));

    // Reset trigger
    no_of_mqtt_bytes = 0;
    mqtt_string.replace(" ", "");
    String cCommand = jsonExtract(mqtt_string, "command");

    debugPrintln(d_error, mqtt_string);
    debugPrintln(d_error, cCommand);
    
    // Test command type
    if (cCommand) {
  
      debugPrintln(d_status, "Validator - Trigger: " + cCommand);
  
      // Command meta data
      String c_UUID_old = jsonExtract(mqtt_string, "com-uuid");
      String cSender    = jsonExtract(mqtt_string, "sender");
      String t_UUID     = jsonExtract(mqtt_string, "task-uuid");
  
      // Test for task. Task is scheduled by time events at 1Hz.
      if (cCommand == "start-task") {

        // Test if task is not ongoing
        if (com_curr.state <= s_undefined) {

          debugPrintln(d_status, "Validator - New task. None in progress");

          clearCommand();
          clearCommandPositions();

          com_curr.com_type   = cCommand;
          com_curr.sender     = cSender;
          com_curr.c_UUID_old = c_UUID_old;
          com_curr.t_UUID     = t_UUID;
          String task_def     = jsonExtract(mqtt_string, "task");
          com_curr.task_name  = jsonExtract(task_def, "name");

          // Initiate state machine
          com_curr.state_old  = s_undefined;
      
          String paramsStruct = jsonExtract(task_def, "params");
          com_curr.speed      = jsonExtract(paramsStruct, "speed");
    
          // Test if command can be handled by the agent
          if (com_curr.task_name == "move-to") {
            String wpStruct = jsonExtract(paramsStruct, "waypoint");
            setCommandPosition(wpStruct);
            printCommandPositions();
            com_curr.state      = s_started;

          } else if (com_curr.task_name == "move-path") {
            String wpsStruct = jsonExtract(paramsStruct, "waypoints");
            bool warning = setCommandPositions(wpsStruct);

            if (warning) {
              debugPrintln(d_error, "ERROR: Commanded path can be too long. Maximum WP's are " + String(max_points));
              sendGeneralInfo("WRN", "Path >= " + String(max_points));
            }
            printCommandPositions();
            com_curr.state      = s_started;

          } else if (com_curr.task_name == "search-area") {
            String wpsStruct = jsonExtract(paramsStruct, "area");
            bool warning = setCommandPositions(wpsStruct);

            if (warning) {
              debugPrintln(d_error, "ERROR: Commanded path can be too long. Maximum WP's are " + String(max_points));
              sendGeneralInfo("WRN", "Path >= " + String(max_points));
            }
            printCommandPositions();
            com_curr.state      = s_started;

          } else if (com_curr.task_name == "go-home") {
            setHomeWP();
            printCommandPositions();
            com_curr.state      = s_started;

          // ... otherwize unsupported command.
          } else {
            debugPrintln(d_error, "Validator - Unsupported task: " + com_curr.task_name);
            sendGeneralInfo("ERR", "Unsupported Task");
            com_curr.state      = s_error;
          }

        } else {
          debugPrintln(d_error, "Validator - Previous task in progress");
          sendGeneralInfo("ERR", "Task in progress");
          // DO NOT SET STATE HERE => TASK IS IN PROGRESS
        }
            
      // The command is a signal. Signals are setting states directlty.
      } else if (cCommand == "signal-task") {

        String sSignal = jsonExtract(mqtt_string, "signal");

        // Test if task is in progress and if signal belongs to that task
        if (com_curr.state > s_started && com_curr.t_UUID == t_UUID) {

          String txt_info   = cSender + ":" + cCommand + ":" + sSignal;
          debugPrintln(d_status, "Validator - Signal meta data: " + txt_info);
          sendGeneralInfo(cCommand, txt_info);
    
          if (sSignal == "$abort") {
            sendSignalAbortACK(c_UUID_old, t_UUID);
            com_curr.state_old = com_curr.state;
            com_curr.state     = s_error;
    
          } else if (sSignal == "$pause") {
            sendSignalPauseACK(c_UUID_old, t_UUID);
            com_curr.state_old = com_curr.state;
            com_curr.state     = s_paused;
    
          } else if (sSignal == "$continue") {
            sendSignalContinueACK(c_UUID_old, t_UUID);
            com_curr.state_old = com_curr.state;
            com_curr.state     = s_running;
    
          } else if (sSignal == "$enough") {
            sendSignalEnoughACK(c_UUID_old, t_UUID);
            com_curr.state_old = com_curr.state;
            com_curr.state     = s_finnished;

          } else {
            sendSignalNACK("Unsupported signal: " + sSignal, c_UUID_old, t_UUID);
            debugPrintln(d_error, "Validator - Unsupported signal!");
            // DO NOT SET STATE HERE => TASK IS IN PROGRESS
          }
        } else {
          sendSignalNACK("No corresponding task", c_UUID_old, t_UUID);
          debugPrintln(d_error, "Validator - Signal does not correspond to running task");
          // DO NOT SET STATE HERE => TASK CAN BE IN PROGRESS
        }
 
      // Level 3,4 handshake integration. Send a pong directly.
      } else if (cCommand == "ping") {
        sendPong(c_UUID_old, t_UUID);
      } else {
        sendSignalNACK("Unsupported command type: " + cCommand, c_UUID_old, t_UUID);
        debugPrintln(d_error, "Validator - Unsupported command type!" + mqtt_string);
      }

    // Unrecognized MQTT message
    } else {
      debugPrintln(d_error, "Validator - Unsupported MQTT/JSON: " + mqtt_string);
    }
  }
}

void executor() {             // Execute and distribute validated tasks.

  // Enable executor to run in main loop but triggered by 1Hz timer thread
  if (!task_exec_1Hz) {return;} 
  else {task_exec_1Hz = false;}

  // Test if task is in progress
  if(com_curr.state != s_undefined) {
//  if(com_curr.state >= s_started && com_curr.state <= s_finnished) {

    debugPrintln(d_status, "Executor - In progress: " + com_curr.com_type);

    // The command is a task specification
    if (com_curr.com_type == "start-task") {

      // Send general info first time
      if (com_curr.state == s_started) {
        String txt_info = com_curr.sender + ":" + com_curr.com_type + ":" + com_curr.task_name + ":" + com_curr.speed;
  
        debugPrintln(d_status, "Executor - Command meta data: " + txt_info);
        sendGeneralInfo(com_curr.com_type, txt_info);  // Debug info

        sim_new_WP = true;          // For Simulator
      }

      // Test if command can be handled by the agent
      if (com_curr.task_name == "move-to") {
        executeMoveToNextWP();

      } else if (com_curr.task_name == "move-path") {
        executeMoveToNextWP();

      } else if (com_curr.task_name == "search-area") {
        executeMoveToNextWP();

      } else if (com_curr.task_name == "go-home") {
        executeMoveToNextWP();

      // ... otherwize unsupported command. Should not be able to happen.
      } else {
        String fail_reason = "Unsupported command: " + com_curr.task_name;
        sendCommandNACK(fail_reason, com_curr.c_UUID_old, com_curr.t_UUID);
        com_curr.state_old = com_curr.state;
        com_curr.state     = s_error;
      }
    }

  } else {
    clearCommand();
  }
}

void executeMoveToNextWP() {  // Execute tasks related to movement. 

  debugPrintln(d_status, "Execute Task Move To");

  // STATE STARTED
  if (com_curr.state == s_started) {
    // Send state when changed
    if (com_curr.state != com_curr.state_old) {
      debugPrintln(d_always, "Execute Task Move To - State: " + states[com_curr.state]);
      sendCommandACK(com_curr.c_UUID_old, com_curr.t_UUID);
      sendTaskStarted(com_curr.c_UUID_old, com_curr.t_UUID);
    }

    com_curr.state_old = com_curr.state;
    com_curr.state     = s_planning;

  // STATE PLANNING
  } else if (com_curr.state == s_planning) {
    // Send state when changed
    if (com_curr.state != com_curr.state_old) {
      debugPrintln(d_always, "Execute Task Move To - State: " + states[com_curr.state]);
      sendTaskPlanning(com_curr.c_UUID_old, com_curr.t_UUID);
    }

    // DO SOME PLANNING STUFF HERE. CURRENTLY NOTHING IS DONE
    // THE COMMAND SHOULD BE TRANSFERED HERE TO THE ACTUAL TASK, E.G. COMMANDED PATH => TRUE/PLANNED PATH
    if (com_curr.task_name == "search-area") {
      makeSearchPlan();
    } else {
      makePlan();
    }

    com_curr.state_old = com_curr.state;
    com_curr.state     = s_running;

  // STATE RUNNING
  } else if (com_curr.state == s_running) {
    // Send state when changed
    if (com_curr.state != com_curr.state_old) {
      debugPrintln(d_always, "Execute Task Move To - State: " + states[com_curr.state]);
      sendTaskRunning(com_curr.c_UUID_old, com_curr.t_UUID);
    }
 
    debugPrintln(d_status, "Execute Task - " + formatGPSData());
//    debugPrintln(d_status, "Execute Task - GPS Quality: " + formatGPSQualityString());

    if (servo_connected) {
      int servo_angle = setServoAngle(getHeadingCorrection());
      debugPrintln(d_always, "Execute Task - " + formatDistanceHeadingData() + ", Servo Angle: " + String(servo_angle) + ", Servo Inverted: " + servo_invert);
    }

    com_curr.state_old = com_curr.state;

    // Approximity Trigger
    if ( abs(getDistance2CurrWP() <= near_WP) ) {

      if (!setNextWP()) {
        com_curr.state  = s_finnished;
      }

      if (servo_connected) {
        // Only a simple "visualizer" for reaching WP
        setServoAngle(-90);
        delay(100);
        setServoAngle(90);
        delay(100);
        setServoAngle(0);
      }
    }

  // STATE PAUSED
  } else if (com_curr.state == s_paused) {
    // Send state when changed
    if (com_curr.state != com_curr.state_old) {
      debugPrintln(d_always, "Execute Task Move To - State: " + states[com_curr.state]);
      sendTaskPaused(com_curr.c_UUID_old, com_curr.t_UUID);
      com_curr.state_old = com_curr.state;

      // Only for simulation - Refactor this
      if (!gps_connected) {
        a_pos_data.spd = 0.0;
      }
    }
    
  // STATE FINNISHED
  } else if (com_curr.state == s_finnished) {
    // Send state when changed
    if (com_curr.state != com_curr.state_old) {
      debugPrintln(d_always, "Execute Task Move To - State: " + states[com_curr.state]);
      sendTaskFinnished(com_curr.c_UUID_old, com_curr.t_UUID);
      clearCommandPositions();
      clearPlannedWaypoints();
    }

    com_curr.state_old = com_curr.state;
    com_curr.state     = s_undefined;

    // Only for simulation - Refactor this
    if (!gps_connected) {
      a_pos_data.spd = 0.0;
    }

  // STATE ERROR
  } else if (com_curr.state == s_error) {
    // Send state when changed
    if (com_curr.state != com_curr.state_old) {
      debugPrintln(d_always, "Execute Task Move To - State: error");
      sendTaskFailed(states[com_curr.state], com_curr.c_UUID_old, com_curr.t_UUID);
      clearCommandPositions();
      clearPlannedWaypoints();
    }

    com_curr.state_old = com_curr.state;
    com_curr.state     = s_undefined;

    // Only for simulation - Refactor this
    if (!gps_connected) {
      a_pos_data.spd = 0.0;
    }

  // STATE NOT RECOGNIZED
  } else {
    debugPrintln(d_error, "Execute Task Move To - Error State: NOT DEFINED STATE!");
    com_curr.state_old = com_curr.state;
    com_curr.state     = s_error;

    // Only for simulation - Refactor this
    if (!gps_connected) {
      a_pos_data.spd = 0.0;
    }
  }

}

// Waypoint handling =================================================================

void makeSearchPlan (){
  debugPrintln(d_always, "Planning - Command positions => Planned Waypoints ...");
  clearPlannedWaypoints();

  // Here the magic shall be made. For now, just a copy of the input points with some deviations.

  // This example uses maximum and minimum GeoPoints to "cover" the whole area
  float lat_max = 0.0;
  float lat_min = 90.0;
  float lon_max = 0.0;
  float lon_min = 180.0;
  float alt_max = -10000.0;
  float alt_min = 10000.0;

  for (int i = 0; i < max_points && i <= command_positions.end_i; i++) {
    GeoPoint gpc = command_positions.ps[i];
    lat_max = max(lat_max, gpc.lat);
    lat_min = min(lat_min, gpc.lat);
    lon_max = max(lon_max, gpc.lon);
    lon_min = min(lon_min, gpc.lon);

    // Alt is not currently used
    alt_max = max(alt_max, gpc.alt);
    alt_min = min(alt_min, gpc.alt);
  }

  static int cuts = 5;
  float lon_diff  = (lon_max-lon_min)/cuts;
  bool up         = true;

  for (int c = 0; c <= cuts; c++) {
    float cut_lon = lon_min + c * lon_diff;

    GeoPoint gpA;
    GeoPoint gpB;

    gpA.lon = cut_lon;
    gpB.lon = cut_lon;
    gpA.alt = alt_min;
    gpB.alt = alt_min;

    if ((c % 2) == 0) { 
      gpA.lat = lat_min;
      gpB.lat = lat_max;
    } else {
      gpB.lat = lat_min;
      gpA.lat = lat_max;
    }

    planned_waypoints.ps[c*2]   = gpA;
    planned_waypoints.ps[c*2+1] = gpB;
  }

  // Add start WP as last WP => Return
  planned_waypoints.ps[cuts * 2 + 2] = planned_waypoints.ps[0];
  
  planned_waypoints.curr_i = 0;
  planned_waypoints.end_i  = cuts * 2 + 2;


//  GeoPoint gpp00;  gpp00.lat  = lat_min; gpp00.lon  = lon_min;                 gpp00.alt  = alt_min;
//  GeoPoint gpp10;  gpp10.lat  = lat_max; gpp10.lon  = lon_min;                 gpp10.alt  = alt_min;
//  GeoPoint gpp11;  gpp11.lat  = lat_max; gpp11.lon  = lon_min + 1 * lon_diff;  gpp11.alt  = alt_min;
//  GeoPoint gpp01;  gpp01.lat  = lat_min; gpp01.lon  = lon_min + 1 * lon_diff;  gpp01.alt  = alt_min;
//
//  GeoPoint gpp02;  gpp02.lat  = lat_min; gpp02.lon  = lon_min + 2 * lon_diff;  gpp02.alt  = alt_min;
//  GeoPoint gpp12;  gpp12.lat  = lat_max; gpp12.lon  = lon_min + 2 * lon_diff;  gpp12.alt  = alt_min;
//  GeoPoint gpp13;  gpp13.lat  = lat_max; gpp13.lon  = lon_max;                 gpp13.alt  = alt_min;
//  GeoPoint gpp03;  gpp03.lat  = lat_min; gpp03.lon  = lon_max;                 gpp03.alt  = alt_min;
//
//  GeoPoint gpp00b; gpp00b.lat = lat_min; gpp00b.lon = lon_min;                 gpp00b.alt = alt_min;
//
//  
//  planned_waypoints.ps[0] = gpp00;
//  planned_waypoints.ps[1] = gpp10;
//  planned_waypoints.ps[2] = gpp11;
//  planned_waypoints.ps[3] = gpp01;
//  planned_waypoints.ps[4] = gpp02;
//  planned_waypoints.ps[5] = gpp12;
//  planned_waypoints.ps[6] = gpp13;
//  planned_waypoints.ps[7] = gpp03;
//  planned_waypoints.ps[8] = gpp00b;

//  planned_waypoints.curr_i  = 0;
//  planned_waypoints.end_i   = 8;
  
//  for (int i = 0; i < max_points; i++) {
//    GeoPoint gp;
//    gp.lat  = command_positions.ps[i].lat;
//    gp.lon  = command_positions.ps[i].lon;
//    gp.alt  = command_positions.ps[i].alt;
//    planned_waypoints.ps[i] = gp;
//  }
//  planned_waypoints.curr_i  = command_positions.curr_i;
//  planned_waypoints.end_i   = command_positions.end_i;

  debugPrintln(d_status, "Planning - Command Positions:");
  printPositions(command_positions);  
  debugPrintln(d_status, "Planning - Planned Waypoints:");
  printPositions(planned_waypoints);

  sendWaypoints();
}

void makePlan() {

  debugPrintln(d_always, "Planning - Command positions => Planned Waypoints ...");
  clearPlannedWaypoints();

  // Here the magic shall be made. For now, just a copy of the input points with some deviations.
  for (int i = 0; i < max_points; i++) {
    GeoPoint gp;
    gp.lat  = command_positions.ps[i].lat;
    gp.lon  = command_positions.ps[i].lon;
    gp.alt  = command_positions.ps[i].alt;
    planned_waypoints.ps[i] = gp;
  }
  planned_waypoints.curr_i  = command_positions.curr_i;
  planned_waypoints.end_i   = command_positions.end_i;

  debugPrintln(d_status, "Planning - Command Positions:");
  printPositions(command_positions);  
  debugPrintln(d_status, "Planning - Planned Waypoints:");
  printPositions(planned_waypoints);  

}

void setHomeWP () {
  GeoPoint gp;
  gp.lat  = A_HOME_LAT;
  gp.lon  = A_HOME_LON;
  gp.alt  = A_HOME_ALT;
  command_positions.end_i  = 0;
  command_positions.curr_i = 0;
  command_positions.ps[0] = gp;
}

void setCommandPosition (String gpStruct) {
  GeoPoint gp = parseGeoPoint(gpStruct);
  command_positions.end_i  = 0;
  command_positions.curr_i = 0;
  command_positions.ps[0]  = gp;
}

bool setCommandPositions (String wpsStruct) {

  bool warning = false;

  clearCommandPositions();

  // Parse and add geopoints to array, max number of points is declared in max_points.
  // Validation of geopoint => TBD.

  command_positions.end_i  = -1;
  command_positions.curr_i = -1;

  for (int cnt = 0; cnt < max_points && command_positions.end_i == -1; cnt++) {
    String    wpStruct  = jsonIndexList(wpsStruct, cnt);
    GeoPoint  gp        = parseGeoPoint(wpStruct);
    command_positions.ps[cnt] = gp;

    // jsonlib do not have a max counter function. 
    // Test that previous point and current is equal => previous point is the end.
    if (cnt > 0 && equal(command_positions.ps[cnt-1], gp) ){
      command_positions.end_i   = (cnt-1);
    }
  }

  // If commanded path has more waypoints than max_path, set end at max_path-1.
  // In this case there will be leftover WP's.
  if (command_positions.end_i == -1) {
    command_positions.end_i = (max_points-1);
    warning = true;
  } else {
    warning = false;
  }

  command_positions.curr_i = 0;

  return warning;
}

bool setNextWP() {
  if (planned_waypoints.end_i > -1 && planned_waypoints.curr_i < planned_waypoints.end_i ) {
    planned_waypoints.curr_i++;
    sim_new_WP = true;  // For Simulator
    return true;

  } else {
    sim_new_WP = false; // For Simulator
    return false;
  }
}

GeoPoint parseGeoPoint (String gpStruct) {
  GeoPoint gp;
  gp.lat = jsonExtract(gpStruct, "latitude").toFloat();
  gp.lon = jsonExtract(gpStruct, "longitude").toFloat();
  gp.alt = jsonExtract(gpStruct, "altitude").toFloat();

  return gp;
}

void clearCommand() {
  com_curr.com_type   = "";
  com_curr.sender     = "";
  com_curr.task_name  = "";
  com_curr.speed      = "";
  com_curr.c_UUID_old = "";
  com_curr.t_UUID     = "";
  com_curr.state_old  = s_undefined;
  com_curr.state      = s_undefined;
}

void clearCommandPositions() {
  debugPrintln(d_status, "Clear Command Positions");

  command_positions.curr_i = -1;
  command_positions.end_i  = -1;
}

void clearPlannedWaypoints() {
  debugPrintln(d_status, "Clear Planned Waypoints");

  planned_waypoints.curr_i = -1;
  planned_waypoints.end_i  = -1;
}

bool equal(GeoPoint gp1, GeoPoint gp2) {
  if (gp1.lat != 0.0 && gp2.lat != 0.0) {
    return (gp1.lat == gp2.lat && gp1.lon == gp2.lon && gp1.alt == gp2.alt);
  } else {
    return false;
  }
}


// HELPERS =============================================================

void setupWiFi() {
  bool primaryAP    = true;
  bool is_connected = false;

  while (!is_connected) {

    if (primaryAP) {
      is_connected = connectToWiFi(wifi_ssid1, wifi_pass1);

    } else {
      is_connected = connectToWiFi(wifi_ssid2, wifi_pass2);
    }

    primaryAP = !primaryAP;
  }
}

bool connectToWiFi(const char* ssid, const char* pass) {
  debugPrintln(d_always, ""); debugPrintln(d_always, "");

  debugPrint(d_always, "WiFi - Trying to connect to ");
  debugPrint(d_always, ssid);
  debugPrintln(d_always, " ...");

  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(client_name);
  WiFi.begin(ssid, pass);

  int no_of_tries = 0;
  int max_tries   = 5;

  while (WiFi.status() != WL_CONNECTED && no_of_tries < max_tries) {
    ledOn();
    delay(50);
    ledOff();
    delay(1950);

    debugPrint(d_always, String(++no_of_tries)); debugPrint(d_always, " ");
  }
  debugPrintln(d_always, "");

  if (WiFi.status() == WL_CONNECTED) {
    // For this simple agent we use ssl connection but do not check certificates
    wifi_client.setInsecure();
  
    ip = WiFi.localIP();
  
    debugPrint(d_always, "WiFi - Connected to ");
    debugPrint(d_always, ssid);
    debugPrint(d_always, " as ");
    debugPrintln(d_always, ip.toString());
    return true;

  } else {
    debugPrint(d_always, "WiFi - Could not connect to ");
    debugPrint(d_always, ssid);
    return false;
  }

}

void setupNTP() {
  debugPrintln(d_always, "NTP - Trying to connect ...");
  time_client.begin(); 
  debugPrintln(d_always, "NTP - Connected");
}

void setupMQTT() {
  debugPrintln(d_always, "MQTT - Trying to connect ...");

  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(mqttListener);
  mqtt_client.setKeepAlive(60);
  mqtt_client.setSocketTimeout(30);
  
  int retries = 0;

  // Loop until connected
  while (!mqtt_client.connected()) {
    if (retries <= 10) {

      // Attempt to connect
      mqtt_client.connect((char*) client_name.c_str(), mqtt_user, mqtt_pass);
      retries++;

      // Wait 5 seconds before retrying
      delay(5000);

      // If still not connected, try to restart contoller
    } else {
      debugPrintln(d_always, "MQTT - Failed to connect! Check credentials.");
      debugPrintln(d_always, "!!! RESTARTING MICROCONTROLLER IN 5 SECONDS !!!");
      delay(5000);
      ESP.restart();
    }
  }
  mqtt_client.setBufferSize(max_mqtt_bytes);

  // Start listening to commands (Tasks/Direct Execution)
  subAgent("exec/command");

  debugPrintln(d_always, "MQTT - Connected as " + client_name);
  sendGeneralInfo("MQTT", "(Re)started");

}

void setupGPS() {
  debugPrintln(d_always, "GPS - Setting up GPS on serial port ...");
  gpsSerial.begin(gps_baud);

  // Clear the list of messages which are sent.
//  nmea.sendSentence(gpsSerial, "$PORZB");

  // Send only RMC and GGA messages.
//  nmea.sendSentence(gpsSerial, "$PORZB,RMC,1,GGA,1");

  // Clear all stored values
  nmea.clear();
  debugPrintln(d_always, "GPS - Done");
}

void setupServo() {
  debugPrintln(d_always, "Servo - Initiating ...");

  servo.attach(servo_pin);
  setServoAngle(90);

  debugPrintln(d_always, "Servo - Done");
}

void setupTriggers() {
  debugPrintln(d_always, "Event Triggers - Setting up");
  timer.setInterval(1000, send_1Hz);
  timer.setInterval(5000, send_0_2Hz);
}

unsigned long getEpoch() {
  time_client.update();
  unsigned long epoch = time_client.getEpochTime();
  return epoch;
}


// Position Helpers ------------------------------------------

void updateAgentPositionGPS() {
  float lat = (nmea.getLatitude()  / 1000000.0);
  float lon = (nmea.getLongitude() / 1000000.0);
  long alt0;
  nmea.getAltitude(alt0);
  float alt = (alt0 / 1000.0);

  float crs = (nmea.getCourse() / 1000.0);
  float hdg = a_pos_data.crs;
  float spd = (nmea.getSpeed()  / 1000.0 * 0.514);

  updateAgentPosition(lat, lon, alt, spd, hdg, crs);

//  debugPrint(d_always, "GPS: nmea alt: " + String(alt0) + " calc alt: " + String(alt, 2));
//  debugPrint(d_always, "GPS: " + formatGPSData());
}

void updateAgentPosition(float lat, float lon, float alt, float spd, float hdg, float crs) {
  // TODO: Validation
  a_pos_data.lat = lat;
  a_pos_data.lon = lon;
  a_pos_data.alt = alt;
  a_pos_data.spd = spd;
  a_pos_data.hdg = hdg;
  a_pos_data.crs = crs;

}

int setServoAngle(float deg) {
  int deg_i;
  int deg_servo;

  // Normal servo can only use -90 to 90 degrees (0-180).
  if (deg > 90)         { deg_i = 90;
  } else if (deg < -90) { deg_i = -90;
  } else {                deg_i = (int)deg;
  }

  // Normalize actual servo action to wanted
  int max_angle_norm = 180 + (180 - servo_angle_max);
  int min_angle_norm = -servo_angle_min;
  
  // Remap +/- 90 deg to servo output
  if (servo_invert) {
    deg_servo = map(deg_i, -90, 90, max_angle_norm, min_angle_norm);
  } else {
    deg_servo = map(deg_i, -90, 90, min_angle_norm, max_angle_norm);
  }

  if      (deg_servo > 180) deg_servo = 180;
  else if (deg_servo < 0)   deg_servo = 0;
  else                      deg_servo = deg_servo;

  servo.write(deg_servo);
  delay(100);

  return deg_servo;
}


// Identifiers ---------------------------------------------------

String getUniqueName() {              // Create a unique name for the agent composed by input id + three last positions from the mac address.
  String mc = getMAC();
  return a_base_name + "_" + mc.substring(mc.length() - 3);
}

String getMAC() {                     // Get the mac address from the network card.
  uint8_t mac[6];
  char result[14];
  WiFi.macAddress(mac);

  snprintf( result, sizeof( result ), "%02x%02x%02x%02x%02x%02x", mac[ 0 ], mac[ 1 ], mac[ 2 ], mac[ 3 ], mac[ 4 ], mac[ 5 ] );

  return String( result );
}

String get_UUID() {                   // Create a unique UUID . NOTE: Do not set randomSeed if not using a random source.

  String idA1 = String(random(0x1000, 0xFFFF), HEX);
  String idA2 = String(random(0x1000, 0xFFFF), HEX);
  String idB  = String(random(0x1000, 0xFFFF), HEX);
  String idC  = String(random(0x1000, 0xFFFF), HEX);
  String idD  = String(random(0x1000, 0xFFFF), HEX);
  String idE1 = String(random(0x1000, 0xFFFF), HEX);
  String idE2 = String(random(0x1000, 0xFFFF), HEX);
  String idE3 = String(random(0x1000, 0xFFFF), HEX);

  return idA1 + idA2 + "-" + idB + "-" + idC + "-" + idD + "-" + idE1 + idE2 + idE3;
}

// Onboard LED Status ----------------------------------------------------

void ledOn() {                        // Turn on the onboard LED
  digitalWrite(LED_BUILTIN, LOW);
}

void ledOff() {                       // Turn off the onboard LED
  digitalWrite(LED_BUILTIN, HIGH);
}

void ledStatus(int no_of_flashes) {   // Shows status flashes on the onboard LED. Used as a status/debug monitor
  debugPrintln(d_status, "LED Status: " + String(no_of_flashes) + " blinks");

  ledOff();
  delay(500);
  for (int i = 0; i < (no_of_flashes * 2) ; i++) {

    if( (i & 1) == 0 ) {
      ledOn();
    } else {
      ledOff();
    }

    delay(100);
  }
}


// Debug Helpers -----------------------------------------

String formatGPSData() {
  return "lat: " + String(a_pos_data.lat,6) + ", " + 
         "lon: " + String(a_pos_data.lon,6) + ", " + 
         "alt: " + String(a_pos_data.alt,2) + ", " + 
         "crs: " + String(a_pos_data.crs,1) + ", " + 
         "spd: " + String(a_pos_data.spd,1);
}

String formatGPSQualityString() {
  // MicroNMEA uses integers for calculating HDOP => Divide by 10 to get actual HDOP.

  float  hdop = formatGPS_HDOP();
  int    sats = nmea.getNumSatellites();
  String data = "HDOP: " + String(hdop, 1) + ", Sats: " + sats;

  if (hdop <= 1) {
    return "Ideal (" + data + ")";

  } else if (hdop > 1  && hdop <= 2 ) {
    return "Excellent (" + data + ")";
  
  } else if (hdop > 2  && hdop <= 5 ) {
    return "Good (" + data + ")";
  
  } else if (hdop > 5  && hdop <= 10) {
    return "Moderate (" + data + ") - Try better position";
  
  } else if (hdop > 10 && hdop <= 20) {
    return "Fair (" + data + ") - Do not use";
  
  } else {
    return "Poor (" + data + ") - Does the GPS work? Change of antenna?";
  }
  
}

float formatGPS_HDOP() {
  // MicroNMEA uses integers for calculating HDOP => Divide by 10 to get actual HDOP.
  float hdop = nmea.getHDOP()/10.0;
  return hdop;
}

String formatDistanceHeadingData() {
  
  float dist      = getDistance2CurrWP    ();
  float headCorr  = getHeadingCorrection  ();       // Correction is negative error

  return ("Distance: " + String(dist, 2) + ", Heading Correction: " + String(headCorr, 1) );
}

String formatGeoPoint(GeoPoint gp) {
  return "GeoPoint - lat: " + String(gp.lat,6) + ", lon: " + String(gp.lon,6) + ", alt: " + String(gp.alt,1);
}

void printWaypoints() {
  printPositions(planned_waypoints);  
}

void printCommandPositions() {
  printPositions(command_positions);  
}

void printPositions(Positions poses) {
  for (int cnt = 0; cnt <= poses.end_i && cnt < (max_points-1) ; cnt++) {
    String meta = "";
    if (cnt == poses.curr_i) meta += "NEXT ";
    if (cnt == poses.end_i ) meta += "LAST ";

    GeoPoint gp = poses.ps[cnt];
    debugPrintln(d_always, String(cnt) + " => " + formatGeoPoint(gp) + ", meta: " + meta);
  }
}


void debugPrint(int level, String text) {
  if (level >= d_threshold) {
    Serial.print(text);
  }
}

void debugPrintln(int level, String text) {
  if (level >= d_threshold) {
    Serial.println();       // Make space
    Serial.println(text);
  }
}
