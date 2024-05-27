#pragma once
#include<vector>
#include<tuple>
#include<mqtt/async_client.h>
#include"include/json.hpp"
#include"config.h" // Defaults and configuration values for agent

// GPS position (latitude, longitude, altitude)
using geopoint = std::tuple<double, double, double>;
// geopoint without altitude
using surface_geopoint = std::tuple<double, double>;

// Stores Agent's sensor info
class GpsClient
{
public:
    GpsClient() { randomize_start_pos(); }

    // Home position, go-home returns to these coordinates
    double lat_home{GpsConfig::LATITUDE};
    double lon_home{GpsConfig::LONGITUDE};
    const double alt_home{GpsConfig::ALTITUDE};
    // Current position, initialized by randomize_start_pos()
    double lat;
    double lon;
    double alt{alt_home};
    double heading{GpsConfig::DUMMY_HEADING};
    double course{GpsConfig::DUMMY_COURSE};
    double speed{0.0};

private:
    // Randomly offset lat and lon so that agents do not start inside eachother
    void randomize_start_pos()
    {
        constexpr double r = 400.0 / 111300.0; // 100 meters
        std::random_device rand_eng;
        std::mt19937 rand_gen(rand_eng());
        std::uniform_real_distribution<> rand_dist(0.0, 1.0);
        const double u = rand_dist(rand_gen);
        const double v = rand_dist(rand_gen);
        const double w = r * std::sqrt(u);
        constexpr double pi = 3.14159265358979323846;
        const double t = 2 * pi * v;
        const double x = w * std::cos(t);
        const double y1 = w * std::sin(t);
        const double x1 = x / std::cos(lat);
        lat = lat_home = GpsConfig::LATITUDE + y1;
        lon = lon_home = GpsConfig::LONGITUDE + x1;
    }
};

// Agent and task configuration
class Logic
{
public:
    Logic(const std::string & name) : name(name + '_' + id_generator()) {};

    // Agent variables
    const std::string name;
    const std::string type{AgentConfig::TYPE};
    const std::string description{AgentConfig::DESCRIPTION};
    const std::string domain{AgentConfig::DOMAIN};
    const std::string sim_real{AgentConfig::SIM_REAL};
    const std::vector<std::string> level{AgentConfig::LEVEL};
    // Ticks per second for sending data
    // Consider sending data of different priority at different intervals instead
    const float rate{AgentConfig::UPDATE_RATE};
    const std::string uuid{create_uuid()};
    // List of which tasks agent is capable of
    const nlohmann::json tasks_available{AgentConfig::TASKS_AVAILABLE};

    // Task variables
    bool task_running{false};
    bool task_pause_flag{false};
    std::string task_running_uuid{};
    // target as (latitude, longitude, altitude) tuple.
    geopoint task_target{};
    std::vector<nlohmann::json> path{};

    // Create v4 UUID for Agent and messages (uuid_v4 library)
    std::string create_uuid() const
    {
        UUIDv4::UUIDGenerator<std::mt19937_64> uuid_gen;
        return uuid_gen.getUUID().str();
    }

private:
    // Create random id (default lowercase alphanumeric, not unique)
    std::string id_generator(unsigned int size = 4, std::string chars = "abcdefghijklmnopqrstuvwxyz0123456789") const
    {
        std::random_device rand_eng;
        std::uniform_int_distribution<int> uniform_int(0, chars.length() - 1);
        std::string id{};

        for (unsigned int i = 0; i < size; i++)
            id += chars.at(uniform_int(rand_eng));
        
        return(id);
    }
};

// Contains Agent's MQTT client and related config
class MqttClient
{
public:
    MqttClient(const std::string& agent_name, const std::string broker_address) : 
        base_topic{MqttConfig::WARAPS_TOPIC_BASE + '/' + agent_name},
        listen_topic{MqttConfig::WARAPS_TOPIC_BASE + '/' + agent_name + MqttConfig::WARAPS_LISTEN_TOPIC},
        client(broker_address, "") { };
    const std::string base_topic;
    const std::string listen_topic;
    mqtt::async_client client;
};
