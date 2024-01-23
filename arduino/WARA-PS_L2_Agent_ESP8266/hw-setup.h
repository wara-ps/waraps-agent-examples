/*
 * Optional HW is declared in this header file.
 */

// GPS SETTINGS ==================================
#define   GPS_CONNECTED     false     // Set to true if gps is used and connected to gps_RX_pin pins. Detaches the simulator when set to true.
#define   GPS_RX_PIN        D2        // RX pin for connecting to GPS TX pin.
#define   GPS_TX_PIN        D1        // TX pin for connecting to GPS RX pin. Not currently used.
#define   GPS_BAUD          9600      // Baud rate for receiving nmea from GPS.


// SERVO SETTINGS ==================================
#define   SERVO_CONNECTED   false     // Connect servo to show the direction to current waypoint.
#define   SERVO_PIN         D4        // Output pin for servo signal.
#define   SERVO_ANGLE_MAX   150       // Actual maximum servo angle 0-180.
#define   SERVO_ANGLE_MIN   30        // Actual minimum servo angle 0-180.
#define   SERVO_INVERT      true      // Positive value => left action => invert = true.
