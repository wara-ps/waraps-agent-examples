#include<iostream>
#include<thread>
#include<chrono>
#include<vector>
#include<cstring>
#include<random>
#include"agent.h"
#include"config.h"
#include"mqtt/async_client.h"

// Periodically send sensor data to MQTT broker
void agent_thread(const std::string & agent_name)
{
    // Connect to broker, using SSL or no encryption depending on TLS setting
    std::string broker_address;
    if (MqttConfig::WARAPS_TLS_CONNECTION)
        broker_address = "ssl://" + MqttConfig::WARAPS_BROKER + ":" + std::to_string(MqttConfig::WARAPS_PORT);
    else
        broker_address = "tcp://" + MqttConfig::WARAPS_BROKER + ":" + std::to_string(MqttConfig::WARAPS_PORT);
    Agent my_agent{agent_name, broker_address};

    // Wait for random duration to avoid agents using the network simultaneously
    std::random_device rand_eng;
    std::uniform_int_distribution<int> uniform_int(0, 500);
    std::chrono::duration<float> random_wait_seconds(uniform_int(rand_eng) / 1000.0f);
    std::this_thread::sleep_for(random_wait_seconds);

    // Print agent name and starting position
    my_agent.print_data();
    
    // Main loop. Receive commands and send sensor data at a regular interval.
    std::chrono::duration<float> wait_seconds(1.0f / my_agent.logic.rate);
    while (1)
    {
        my_agent.check_task();
        my_agent.send_data();
        std::this_thread::sleep_for(wait_seconds);
    }
}

int main(int argc, char* argv[])
{
    // Read environment variables for MQTT connection
    MqttConfig::read_from_env();

    // Read input arguments (agent name, number of units to create)
    size_t units{4};
    std::string name{AgentConfig::NAME};
    for (int i = 1; i < argc - 1; i++)
    {
        if (!std::strcmp(argv[i], "-u") or !std::strcmp(argv[i], "--units"))
            units = std::stoi(argv[++i]);
        else if (!std::strcmp(argv[i], "-n") or !std::strcmp(argv[i], "--name"))
            name = std::string(argv[++i]);
        else
        {
            std::cout << argv[0] << " received incorrect argument. Arguments: -n, --name, -u, --unit\n";
            return -1;
        }
    }

    // Create one thread for each Agent
    std::vector<std::thread> threads{};
    for (size_t i = 0; i < units; i++)
        threads.push_back(std::thread(agent_thread, name));
    
    threads.at(0).join(); // Run agent threads forever
}
