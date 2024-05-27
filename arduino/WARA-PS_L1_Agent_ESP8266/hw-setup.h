/*
 * Optional HW is declared in this header file.
 */

// GPS SETTINGS ==================================
#define GPS_CONNECTED   false     // Set to true if gps is used and connected to gps_RX_pin pins. Detaches the simulator when set to true.
#define GPS_RX_PIN      D2        // RX pin for connecting to GPS TX pin.
#define GPS_TX_PIN      D1        // Not currently used.
#define GPS_BAUD        9600      // Baud rate for receiving nmea from GPS.
