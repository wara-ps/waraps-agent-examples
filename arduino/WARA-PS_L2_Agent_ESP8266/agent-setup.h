/*
 * Agent declaration/setup
 */

#define A_TOPIC_BASE  "waraps/unit"          // Base topic that makes the top for all other sub topics for an agent.

#define A_BASE_NAME   "wESPer"               // Base name of agent.
#define A_TYPE        "esp8266"              // Type of agent.
#define A_DESCR       "L1_agent_example"     // Description of agent.
#define A_DOMAIN      "air"                  // The domain for the agent (lowercase): [ground, surface, subsurface, air].
#define A_SIM_REAL    "simulation"           // Is the agent simulated or real: [simulation, real].
#define A_LEVEL       2                      // Define agent level.

#define A_HOME_LAT    58.391957              // WGS84 latitude       in decimal degrees + => north - => south.
#define A_HOME_LON    15.565350              // WGS84 longitude      in decimal degrees + => east  - => west.
#define A_HOME_ALT    90.0                   // Ellipsoid altitude   in decimal meters.

#define A_DUMMY_HDG   47.7                   // Dummy heading or heading for at static agent.
#define A_DUMMY_CRS   47.5                   // Dummy course.
#define A_DUMMY_SPD   0.9                    // Dummy speed.

// Camera example source. Shall be a rtsp but the Core System can also handle rtmp streams (contact support).
#define A_CAM_URL     "rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov"
