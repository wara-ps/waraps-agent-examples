# 1. Install Paho requirements
apt-get update && apt-get install -y \
    g++ \
    cmake \
    git \
    libssl-dev

# 2. Build Paho C library (required for Paho C++)
git clone https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
-DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
cmake --build build/ --target install
ldconfig

# 3. Build Paho C++ library
git clone https://github.com/eclipse/paho.mqtt.cpp
cd paho.mqtt.cpp

cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON
cmake --build build/ --target install
ldconfig