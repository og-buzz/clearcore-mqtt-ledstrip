#!/bin/bash
# Host-side tests. Needs g++ and, for the MQTT end-to-end test,
#   pip install amqtt paho-mqtt
# 1) Decodes the SPI byte stream each sketch produces as a WS2811 receiver
#    would (pulse widths, latch, byte order, power limiter).
# 2) Runs the MQTT sketch against a local broker (amqtt) and drives it with
#    paho-mqtt: connect, LWT, subscriptions, every command, keep-alive,
#    reconnect after a broker restart.
set -e
cd "$(dirname "$0")"
mkdir -p build
cp ../ClearCore_WS2811_COB/ClearCore_WS2811_COB.ino   build/sketch_cob.ino
cp ../ClearCore_WS2811_MQTT/ClearCore_WS2811_MQTT.ino build/sketch_mqtt.ino
# point the sketch at the local broker for the end-to-end test
sed -i 's/^#define MQTT_BROKER_IP .*/#define MQTT_BROKER_IP      "127.0.0.1"/' build/sketch_mqtt.ino

echo "== COB sketch: WS2811 encoder / limiter"
g++ -std=gnu++11 -O1 -Istub  '-DSKETCH_PATH="build/sketch_cob.ino"'  test_cob_encoder.cpp  -o build/test_cob  -lm
./build/test_cob | tail -1

echo "== MQTT sketch: WS2811 encoder / limiter"
g++ -std=gnu++11 -O1 -Istub2 '-DSKETCH_PATH="build/sketch_mqtt.ino"' test_mqtt_encoder.cpp -o build/test_mqtt -lm -lpthread
./build/test_mqtt | tail -1

echo "== MQTT sketch: end-to-end against a local broker"
g++ -std=gnu++11 -O1 -Istub2 '-DSKETCH_PATH="build/sketch_mqtt.ino"' harness_mqtt.cpp -o build/harness_mqtt -lm -lpthread
python3 mqtt_test.py 2>/dev/null | grep -E "^(PASS|FAIL|RESULT)"
