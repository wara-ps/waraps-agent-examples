#pragma once
#include<string>
#include<vector>
#include"include/json.hpp"
#include"include/uuid_v4.h"

// Configuration values for Agent objects.
namespace AgentConfig
{
    // Base name of agent. Will be overwritten if -n or --name command line argument used
    const std::string NAME{"cpp_l1_agent"};
    // Type of agent.
    const std::string TYPE{"air"};
    // Description of agent.
    const std::string DESCRIPTION{"cpp_l1_agent_example"};
    // The domain for the agent (lowercase): [ground, surface, subsurface, air].
    const std::string DOMAIN{"air"};
    // Is the agent simulated or real: [simulation, real].
    const std::string SIM_REAL{"simulation"};
    // Agent level (e.g. sensor, direct execution)
    const std::vector<std::string> LEVEL{"sensor"};
    // Update rate for sending sensor data (times/second)
    const float UPDATE_RATE{3.0f};
};

// Initial GPS info
namespace GpsConfig
{
    // Initialize at position on Gränsö
    const double LATITUDE{57.76115009154693}; // WGS84 latitude in decimal degrees + => north - => south
    const double LONGITUDE{16.684087827200084}; // WGS84 longitude in decimal degrees + => east  - => west
    const double ALTITUDE{60}; // Ellipsoid altitude in decimal
    const double DUMMY_HEADING{0.0}; // Dummy heading or heading for a static agent
    const double DUMMY_CAM_HEADING{0.0}; // Dummy heading
    const double DUMMY_COURSE{0.0}; // Dummy course
    const double SPEED_SLOW{1.5}; // Meter/second ~= walking pace human
    const double SPEED_STANDARD{SPEED_SLOW * 2};
    const double SPEED_FAST{SPEED_STANDARD * 2};
}

// Default MQTT settings, if corresponding environment variables exist they will
// be overwritten by read_from_env();
namespace MqttConfig
{
    const std::string WARAPS_TOPIC_BASE{"waraps/unit/" + AgentConfig::DOMAIN + "/" + AgentConfig::SIM_REAL};
    std::string WARAPS_BROKER{"localhost"};
    int WARAPS_PORT{1883};
    const std::string BASE_VIDEOSERVER_URL{"wss://ome.waraps.org:3334/app"};
    bool WARAPS_TLS_CONNECTION{false};
    std::string WARAPS_USERNAME{};
    std::string WARAPS_PASSWORD{};

    // Read environment variables (for Docker).
    // Defaults from MqttConfig are used for any missing env variables.
    void read_from_env()
    {
        if(const char* env_read = std::getenv("WARAPS_BROKER"))
            WARAPS_BROKER = env_read;
        if(const char* env_read = std::getenv("WARAPS_PORT"))
            WARAPS_PORT = std::stoi(env_read);
        if(const char* env_read = std::getenv("WARAPS_TLS_CONNECTION"))
            WARAPS_TLS_CONNECTION = (!std::strcmp(env_read, "TRUE") ? true : false);
        if(const char* env_read = std::getenv("WARAPS_USERNAME"))
            WARAPS_USERNAME = env_read;
        if(const char* env_read = std::getenv("WARAPS_PASSWORD"))
            WARAPS_PASSWORD = env_read;
    }
}
