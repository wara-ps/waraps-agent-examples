#pragma once
#include<algorithm>
#include<chrono>
#include<cmath>
#include<iostream>
#include<string>
#include<tuple>
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
        send_direct_execution_info();
    }

    void print_data() const
    {
        std::cout << "Agent " << logic.name << " starting at (lat, lon, alt): " <<
            "(" << gps.lat << ", " << gps.lon << ", " << gps.alt << ")\n";
    }

    // Check if there is a task to perform and send feedback if it's finished
    void check_task()
    {
        if (logic.task_running && !logic.task_pause_flag)
        {
            move_to_target(logic.task_target);

            if (!logic.task_running)
            {
                gps.speed = 0;

                const nlohmann::json json_msg = {
                    {"agent-uuid", logic.uuid},
                    {"com-uuid", logic.create_uuid()},
                    {"status", "finished"},
                    {"task-uuid", logic.task_running_uuid},
                };
                logic.task_running_uuid = "";

                publish(mqtt_client.base_topic + "/exec/feedback", json_msg.dump());
            }
        }
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

        // Triggered when a message is published on topics agent subscribes to
        mqtt_client.client.set_message_callback([this](mqtt::const_message_ptr msg)
        {
            try
            {
                const std::string msg_str{msg->get_payload_str()};
                nlohmann::json msg_json{nlohmann::json::parse(msg_str)};
                // nlohmann makes a nested array so we have to take first element
                msg_json = msg_json[0];
                const std::string com_uuid = msg_json["com-uuid"];

                if (msg_json["command"] == "start-task")
                {
                    const std::string task_uuid = msg_json["task-uuid"];
                    const nlohmann::json task = msg_json["task"];

                    // start-task response
                    nlohmann::json msg_res_json = {
                        {"agent-uuid", logic.uuid},
                        {"com-uuid", logic.create_uuid()},
                        {"fail-reason", ""},
                        {"response", ""},
                        {"response-to", com_uuid},
                        {"task-uuid", task_uuid}
                    };

                    // Feedback message
                    nlohmann::json msg_feed_json = {
                        {"agent-uuid", logic.uuid},
                        {"com-uuid", logic.create_uuid()},
                        {"status", ""},
                        {"task-uuid", task_uuid}
                    };

                    if (is_task_supported(task) and not logic.task_running)
                    {
                        if (task["name"] == "move-to")
                        {
                            logic.task_running = true;
                            logic.task_running_uuid = task_uuid;
                            msg_res_json["response"] = "running";
                            msg_res_json["fail-reason"] = "";
                            msg_feed_json["status"] = "running";
                            const geopoint target{
                                task["params"]["waypoint"]["latitude"], 
                                task["params"]["waypoint"]["longitude"], 
                                task["params"]["waypoint"]["altitude"]};
                            logic.task_target = target;

                            initialize_speed(task);
                        }

                        else if (task["name"] == "go-home")
                        {
                            logic.task_running = true;
                            logic.task_running_uuid = task_uuid;
                            msg_res_json["response"] = "running";
                            msg_res_json["fail-reason"] = "";
                            msg_feed_json["status"] = "running";

                            std::get<0>(logic.task_target) = gps.lat_home;
                            std::get<1>(logic.task_target) = gps.lon_home;
                            std::get<2>(logic.task_target) = gps.alt_home;

                            initialize_speed(task);
                        }

                        else if (task["name"] == "move-path")
                        {
                            logic.task_running = true;
                            logic.task_running_uuid = task_uuid;
                            msg_res_json["response"] = "running";
                            msg_res_json["fail-reason"] = "";
                            msg_feed_json["status"] = "running";

                            // Build up path queue for agent to follow
                            for (const auto & point: task["params"]["waypoints"])
                                logic.path.push_back(point);

                            // Set first point as current target and remove it from the queue
                            std::get<0>(logic.task_target) = logic.path[0]["latitude"];
                            std::get<1>(logic.task_target) = logic.path[0]["longitude"];
                            std::get<2>(logic.task_target) = logic.path[0]["altitude"];
                            logic.path.erase(logic.path.begin());

                            initialize_speed(task);
                        }

                        else if (task["name"] == "search-area")
                        {
                            logic.task_running = true;
                            logic.task_running_uuid = task_uuid;
                            msg_res_json["response"] = "running";
                            msg_res_json["fail-reason"] = "";
                            msg_feed_json["status"] = "running";

                            std::vector<geopoint> area_points{};
                            for (const auto & point: task["params"]["area"])
                            {
                                const geopoint current{
                                    point["latitude"], point["longitude"], point["altitude"]};
                                area_points.push_back(current);
                            }

                            plan_area_search(area_points);

                            initialize_speed(task);
                        }
                    }

                    else
                    {
                        if (logic.task_running)
                            msg_res_json["fail-reason"] = "A task is already running";
                        else // Task not supported
                            msg_res_json["fail-reason"] = "Task is not supported";
                        msg_res_json["response"] = "failed";
                        msg_feed_json["status"] = "failed";
                    }

                    mqtt_client.client.publish(mqtt_client.base_topic + "/exec/response", msg_res_json.dump());
                    mqtt_client.client.publish(mqtt_client.base_topic + "/exec/feedback", msg_feed_json.dump());
                }

                // Command that affects running task
                else if (msg_json["command"] == "signal-task")
                {
                    const std::string signal = msg_json["signal"];
                    const std::string signal_task_uuid = msg_json["task-uuid"];

                    nlohmann::json msg_res_json = {
                        {"agent-uuid", logic.uuid},
                        {"com-uuid", logic.create_uuid()},
                        {"fail-reason", ""},
                        {"response", ""},
                        {"response-to", com_uuid},
                        {"task-uuid", logic.task_running_uuid}
                    };

                    // Feedback message
                    nlohmann::json msg_feed_json = {
                        {"agent-uuid", logic.uuid},
                        {"com-uuid", logic.create_uuid()},
                        {"status", ""},
                        {"task-uuid", logic.task_running_uuid}
                    };

                    // Task signals
                    if (logic.task_running_uuid == signal_task_uuid)
                    {
                        if (signal == "$abort")
                        {
                            msg_feed_json["status"] = "aborted";
                            reinstate_agent_variables();
                        }

                        else if (signal == "$enough")
                        {
                            msg_feed_json["status"] = "enough";
                            reinstate_agent_variables();
                        }

                        // Pause should set gps.speed to 0
                        else if (signal == "$pause")
                        {
                            msg_feed_json["status"] = "paused";
                            logic.task_pause_flag = true;
                        }

                        else if (signal == "$continue")
                        {
                            msg_feed_json["status"] = "running";
                            logic.task_pause_flag = false;
                        }

                        msg_res_json["response"] = "ok";
                    }

                    // Incorrect task id, reject task signal
                    else
                    {
                        msg_res_json["fail-reason"] = "Invalid task-uuid";
                        msg_res_json["response"] = "failed";
                        msg_feed_json["status"] = "failed";
                    }

                    mqtt_client.client.publish(mqtt_client.base_topic + "/exec/response", msg_res_json.dump());
                    mqtt_client.client.publish(mqtt_client.base_topic + "/exec/feedback", msg_feed_json.dump());
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error: Message callback for agent " << logic.name << " caught exception: " << e.what() << std::endl;
            }
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

        // Subscribe to topic for commands.
        // Might want to error check token_ptr return value of async_client::subscribe.
        constexpr int qos{1}; // Use QOS 1 for subscription to commands
        mqtt_client.client.subscribe(mqtt_client.listen_topic, qos)->wait();
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

    // Set current speed of agent according to task msg
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

    // Reset agent's task variables
    void reinstate_agent_variables()
    {
        logic.task_running = false;
        logic.task_pause_flag = false;
        logic.path.clear();
        gps.speed = 0.0;
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

    // Publish the direct execution information of the agent to its topic
    void send_direct_execution_info()
    {
        const nlohmann::json json_msg = {
            {"type", "DirectExecutionInfo"},
            {"name", logic.name},
            {"rate", logic.rate},
            {"stamp", now()},
            // tasks_available ends up being wrapped in an array for some
            // reason so we have to extract the first value in the array
            {"tasks-available", logic.tasks_available[0]}
        };
        publish(mqtt_client.base_topic + "/direct_execution_info", json_msg.dump());
    }

    // Arithmetic mean radius of earth in kilometers for Haversine calculations
    const double earth_radius{6371.0088};
    
    // Calculate the distance between two coordinates in kilometers
    double haversine(surface_geopoint current, surface_geopoint target) const
    {
        // Get the latitude and longitude differences in radians
        const double dLat = to_radians(std::get<0>(target) - std::get<0>(current));
        const double dLon = to_radians(std::get<1>(target) - std::get<1>(current));

        // Apply the haversine formula
        double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                    std::cos(to_radians(std::get<0>(current))) * std::cos(to_radians(std::get<0>(target))) *
                    std::sin(dLon / 2) * std::sin(dLon / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        double d = earth_radius * c;

        return d;
    }

    // Calculate destination from origin given traveled distance and bearing
    surface_geopoint inverse_haversine(surface_geopoint origin, double distance, double bearing)
    {
        // Convert the inputs to radians
        double lat1 = to_radians(std::get<0>(origin));
        double lon1 = to_radians(std::get<1>(origin));
        double d = distance / earth_radius;

        // Apply the inverse haversine formula
        double lat2 = std::asin(std::sin(lat1) * std::cos(d) + std::cos(lat1) * std::sin(d) * std::cos(bearing));
        double lon2 = lon1 + std::atan2(std::sin(bearing) * std::sin(d) * std::cos(lat1), std::cos(d) - std::sin(lat1) * std::sin(lat2));

        // Return the coordinate in degrees
        return surface_geopoint{to_degrees(lat2), to_degrees(lon2)};
    }

    double to_radians(double degrees) const
    {
        constexpr double pi = 3.14159265358979323846;
        constexpr double ratio = pi / 180;
        return degrees * ratio;
    }

    double to_degrees(double radians) const
    {
        constexpr double pi = 3.14159265358979323846;
        constexpr double ratio = 180 / pi;
        return radians * ratio;
    }
    
    // Move agent to the target position in Latitude, Longitude and Altitude
    void move_to_target(const geopoint & target)
    {
        const surface_geopoint current_no_alt{ gps.lat, gps.lon, };
        const surface_geopoint target_no_alt{
            std::get<0>(target), std::get<1>(target) };

        // Distance in kilometers
        const double distance = haversine(current_no_alt, target_no_alt);

        const double speed_km_per_second = gps.speed / 1000.0;
        const double speed = speed_km_per_second / logic.rate;

        // Check if close to target (hardcoded but should depend on movement speed)
        if (distance <= 0.005)
        {
            gps.lat = std::get<0>(target_no_alt);
            gps.lon = std::get<1>(target_no_alt);
            if (logic.path.empty()) // no path
                logic.task_running = false;
            else // there is a path
            {
                std::get<0>(logic.task_target) = logic.path[0]["latitude"];
                std::get<1>(logic.task_target) = logic.path[0]["longitude"];
                std::get<2>(logic.task_target) = logic.path[0]["altitude"];
                logic.path.erase(logic.path.begin());
            }
        }
        else
        {  
            const double bearing = bearing_radians(current_no_alt, target_no_alt);
            gps.heading = to_degrees(bearing);
            gps.course = to_degrees(bearing);
            surface_geopoint new_location = inverse_haversine(current_no_alt, speed, bearing);

            // Change position of the Agent
            gps.lat = std::get<0>(new_location);
            gps.lon = std::get<1>(new_location);
        }
        // Code for altitude:
        const double distance_height = std::get<2>(logic.task_target) - gps.alt;

        if (abs(distance_height) > gps.speed)
        {
            const double speed_m_per_rate = gps.speed / logic.rate;
            if (distance_height < gps.speed)
                gps.alt -= speed_m_per_rate;
            else if (distance_height > gps.speed)
                gps.alt += speed_m_per_rate;
        }
        else
            gps.alt = std::get<2>(logic.task_target);
    }

    // Plan a zig-zag path for the agent to move over an area
    void plan_area_search(const std::vector<geopoint> & area_points)
    {
        // Make use of maximum and minimum GeoPoints to "cover" the whole area
        double lat_max{0.0};
        double lat_min{90.0};
        double lon_max{0.0};
        double lon_min{180.0};
        double alt_max{-10000.0};
        double alt_min{10000.0};

        for (size_t i{0}; i < area_points.size(); i++)
        {
            const geopoint & current_point = area_points[i];
            const double latitude = std::get<0>(current_point);
            const double longitude = std::get<1>(current_point);
            const double altitude = std::get<2>(current_point);
            lat_max = std::max(lat_max, latitude);
            lat_min = std::min(lat_min, latitude);
            lon_max = std::max(lon_max, longitude);
            lon_min = std::min(lon_min, longitude);
            alt_max = std::max(alt_max, altitude);
            alt_min = std::min(alt_min, altitude);
        }

        const unsigned int area_divisions{5};
        const double lon_diff = (lon_max - lon_min) / area_divisions;

        // Create travel points to create a zigzag path that cover the whole area
        for (size_t c{0}; c <= area_divisions; c++)
        {
            const double cut_lon = lon_min + (c * lon_diff);

            nlohmann::json geopoint_A{};
            nlohmann::json geopoint_B{};

            if ((c % 2) == 0)
            {
                geopoint_A["latitude"] = lat_min;
                geopoint_B["latitude"] = lat_max;
            }
            else
            {
                geopoint_A["latitude"] = lat_max;
                geopoint_B["latitude"] = lat_min;
            }

            geopoint_A["longitude"] = cut_lon;
            geopoint_B["longitude"] = cut_lon;
            geopoint_A["altitude"] = alt_min;
            geopoint_B["altitude"] = alt_min;
            
            logic.path.push_back(geopoint_A);
            logic.path.push_back(geopoint_B);
        }

        // Set first point as first task target
        logic.task_target = geopoint{
            logic.path[0]["latitude"], logic.path[0]["longitude"], logic.path[0]["altitude"]};
        logic.path.erase(logic.path.begin());
    }

    // Check if the task is supported by the agent
    bool is_task_supported(const nlohmann::json& task)
    {
        const std::string & name{task["name"]};
        // Unwrap nested json and check if task is in tasks_available
        for (const auto & ava_task: logic.tasks_available[0])
        {
            if (name == ava_task["name"])
                return true;
        }
        return false;
    }

    // Calculate bearing between two points
    double bearing_radians(const surface_geopoint & current, const surface_geopoint & target) const
    {
        const double startLat = to_radians(std::get<0>(current));
        const double startLng = to_radians(std::get<1>(current));
        const double destLat = to_radians(std::get<0>(target));
        const double destLng = to_radians(std::get<1>(target));
        const double y = std::sin(destLng - startLng) * std::cos(destLat);
        const double x = std::cos(startLat) * std::sin(destLat) - std::sin(startLat) * std::cos(destLat) * std::cos(destLng - startLng);
        return std::atan2(y, x);
    }
};
