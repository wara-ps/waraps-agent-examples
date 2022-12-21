/* ================================================================================================================================
  Name:         WARA-PS_L1_Agent_ESP8266.ino
  API version:  0.7 (WARA-PS Core System API Specification)
  IDE:          Arduino IDE 1.8.15
  Target:       Tested on WeMos/Lolin D1 Mini but should work on all ESP8266/32/12 development boards 
  Developer:    Combitech AB (Michael Petterstedt) - Librariy examples have been used for inspiration

  DESCRIPTION ---------------------
  This is an example application of a simple sensor (Level 1) with WARA-PS Core System.
  The application demonstrates how to implement the WARA-PS Core System API Specification.

  The agent connects to the WARA-PS Core System, through the MQTT broker, report itself as an agent in the air domain. The 
  properties that are sent are:
  - position
  - speed
  - course
  - heading
  - camera_url
  If GPS is connected and the GPS_CONNECTED variable (in hw-setup.h file) is set to true the agent will 
  update the properties related to a GPS receiver.

  The application can be verified in the WARA-PS Integration Test Tool (https://2021.nodered.waraps.org/ui) 
  where the agents meta data, position etc. can be reviewed.

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

struct pos_struct {
  float       lat             = 0.0;                    // WGS84 latitude       in decimal degrees + => north - => south.
  float       lon             = 0.0;                    // WGS84 longitude      in decimal degrees + => east  - => west.
  float       alt             = 0.0;                    // Ellipsoid altitude   in decimal meters.
  float       spd             = 0.0;                    // Speed over ground    in decimal meters/second.
  float       crs             = 0.0;                    // Course over ground   in decimal degrees, north up.
  float       hdg             = 0.0;                    // Heading              in decimal degrees, north up.
};

typedef struct pos_struct PosData;
PosData       a_pos_data;                               // Storage for actual position/GPS Data.

// WGS84 GeoPoint for position and WP handling -------
struct point {
  float       lat             = 0.0;                    // WGS84 latitude       in decimal degrees + => north - => south.
  float       lon             = 0.0;                    // WGS84 longitude      in decimal degrees + => east  - => west.
  float       alt             = 0.0;                    // Ellipsoid altitude   in decimal meters.
};

typedef struct point GeoPoint;

// MQTT helpers --------------------
const    int            mqtt_msg_wait_ms  = 50;         // Milliseconds to wait for MQTT message to be sent (> 10 is recommended when using servo).
const    int            max_mqtt_bytes    = 4096;

// DEBUG PRINTS ============================================================================
const int     d_threshold     = 2;                      // Threshold level (>=) for output prints

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
int           sim_tick        = 0;                      // Simulator variable
int           max_sim_ticks   = 10;                     // Simulator variable


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
    gpsSerial.begin(gps_baud);
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

  // Synchronize mqtt and time events
  mqtt_client.loop();
  timer.run();

}


// AGENT SENSORS/LISTENERS ===========================================================
// set global variables, e.g. from parsing of GPS etc.

void getAgentPositionSpeedCourseHeading() {

  if (gps_connected) {

    // Get GPS position from serial port
    if (gpsSerial.available()) {

      while (gpsSerial.available()) {
        char c = gpsSerial.read();
  
        if (nmea.process(c)) {
  
          if(nmea.isValid()) {
            updateAgentPositionGPS();
          }
        }
      }
    }
  } else {
    a_pos_data.lat = A_HOME_LAT;
    a_pos_data.lon = A_HOME_LON;
    a_pos_data.alt = A_HOME_ALT;

    a_pos_data.spd = A_DUMMY_SPD;
    a_pos_data.hdg = A_DUMMY_HDG;
    a_pos_data.crs = A_DUMMY_CRS;
  }
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
  
}

void send_0_2Hz() { // Low update rate
  debugPrintln(d_always, "|");

  float update_rate = 0.2; // Hz

  sendSensorInfo(update_rate);
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

  debugPrintln(d_always, "MQTT - Connected as " + client_name);
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
  float lat  = nmea.getLatitude()  / 1000000.0;
  float lon  = nmea.getLongitude() / 1000000.0;
  long alt0;
  nmea.getAltitude(alt0);
  float alt = alt0 / 1000.0;

  float crs = nmea.getCourse() / 1000.0;
  float hdg = a_pos_data.crs;
  float spd = nmea.getSpeed()  / 1000.0 * 0.514;

  updateAgentPosition(lat, lon, alt, spd, hdg, crs);
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

String get_UUID() {                    // Create a unique UUID . NOTE: Do not set randomSeed if not using a random source.

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

String formatGeoPoint(GeoPoint gp) {
  return "GeoPoint - lat: " + String(gp.lat,6) + ", lon: " + String(gp.lon,6) + ", alt: " + String(gp.alt,1);
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
