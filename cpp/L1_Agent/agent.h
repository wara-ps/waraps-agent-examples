#pragma once
#include<chrono>
#include<iostream>
#include<string>
#include"mqtt/async_client.h"
#include"classes.h"

class Agent
{
public:
    Agent(const std::string & name, const std::string & broker_address)
    : logic(name), mqtt_client(logic.name, broker_address)
    {
        try
        {
            register_mqtt_callbacks();
            connect_to_broker();
        }
        catch (const mqtt::exception& exc)
        {
            std::cerr << exc << std::endl;
            exit(1);
        }
    }

    GpsClient gps{};
    Logic logic;
    MqttClient mqtt_client;

    void send_data()
    {
        send_heartbeat();
        send_sensor_info();
        send_position();
        send_speed();
        send_course();
        send_heading();
    }

    void print_data() const
    {
        std::cout << "Agent " << logic.name << " starting at (lat, lon, alt): " <<
            "(" << gps.lat << ", " << gps.lon << ", " << gps.alt << ")\n";
    }

private:
    // Get current Unix time in seconds
    double now()
    {
        // Convert to seconds (clock period numerator divided by denominator)
        constexpr double ratio{
            (double)std::chrono::system_clock::period::num/std::chrono::system_clock::period::den};
        return std::chrono::system_clock::now().time_since_epoch().count() * ratio;
    }

    // Set MQTT callbacks (on_connect, on_message and on_disconnect in Python agent).
    // The callbacks should probably take the error code into account, like in the Python agent.
    void register_mqtt_callbacks()
    {
        // Triggered when the client connects to the broker
        mqtt_client.client.set_connected_handler([this](const std::string& cause) {
            std::cout << "Agent " << logic.name << " connecting to broker: " << cause << std::endl;
        });

        // Triggered when the client gets disconnected from the broker.
        mqtt_client.client.set_connection_lost_handler([this](const std::string&) {
            std::cerr << "Agent " << logic.name << " lost connection to broker." << std::endl;
        });
    }

    // Connect to the broker using the MQTT client
    void connect_to_broker()
    {
        auto connopts = mqtt::connect_options_builder()
                    .user_name(MqttConfig::WARAPS_USERNAME)
                    .password(MqttConfig::WARAPS_PASSWORD)
                    .finalize();
        if (MqttConfig::WARAPS_TLS_CONNECTION)
        {
            auto sslopts = mqtt::ssl_options_builder()
                            .verify(false)
                            .enable_server_cert_auth(false)
                            .error_handler([](const std::string& msg) {
                                std::cerr << "SSL Error: " << msg << std::endl;
                            })
                            .finalize();

            connopts.set_ssl(std::move(sslopts));
        }
        
        mqtt_client.client.connect(connopts)->wait();
    }

    // Publish message on the given topic
    // Might want to error check using return value from async_client::publish()
    void publish(const std::string& topic, const std::string& msg)
    {
        try
        {
            mqtt_client.client.publish(topic, msg);
        }
        catch (const mqtt::exception& exc)
        {
            std::cerr << "Agent::publish() caught exception: " << exc << std::endl;
            exit(-1);
        }
    }

    // TODO: Set current speed of agent according to task msg
    void initialize_speed(const nlohmann::json & task)
    {
        const std::string & speed{task["params"]["speed"]};
        if (speed == "slow")
            gps.speed = GpsConfig::SPEED_SLOW;
        else if (speed == "standard")
            gps.speed = GpsConfig::SPEED_STANDARD;
        else if (speed == "fast")
            gps.speed = GpsConfig::SPEED_FAST;
        else
            gps.speed = GpsConfig::SPEED_STANDARD;
    }

    // Publish the heartbeat information of the agent to its topic
    void send_heartbeat()
    {
        const nlohmann::json json_msg = {
            {"name", logic.name},
            {"agent-model", "rotary.drone"}, // Model for Cesium.
            {"agent-type", logic.type},
            {"agent-description", logic.description},
            {"agent-uuid", logic.uuid},
            {"levels", logic.level},
            {"rate", logic.rate},
            {"stamp", now()},
            {"type", "HeartBeat"},
        };
        publish(mqtt_client.base_topic + "/heartbeat", json_msg.dump());
    }

    // Publish the sensor information of the agent to its topic
    void send_sensor_info()
    {
        const nlohmann::json json_msg = {
            {"name", logic.name},
            {"rate", logic.rate},
            {"sensor-data-provided", {
                "position",
                "speed",
                "course",
                "heading",
            }},
            {"stamp", now()},
            {"type", "SensorInfo"},
        };
        publish(mqtt_client.base_topic + "/sensor_info", json_msg.dump());
    }

    // Publish the position of the agent to its topic
    void send_position()
    {
        const nlohmann::json json_msg = {
            {"latitude", gps.lat},
            {"longitude", gps.lon},
            {"altitude", gps.alt},
            {"type", "GeoPoint"},
        };
        publish(mqtt_client.base_topic + "/sensor/position", json_msg.dump());
    }

    // Publish the speed of the agent to its topic
    void send_speed()
    {
        publish(mqtt_client.base_topic + "/sensor/speed", std::to_string(gps.speed));
    }

    // Publish the course of the agent to its topic
    void send_course()
    {
        publish(mqtt_client.base_topic + "/sensor/course", std::to_string(gps.course));
    }

    // Publish the heading of the agent to its topic
    void send_heading()
    {
        publish(mqtt_client.base_topic + "/sensor/heading", std::to_string(gps.heading));
    }
};
