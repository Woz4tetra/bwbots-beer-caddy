read -s -p "Please enter the wifi password: " USER_INPUT_WIFI_PASSWORD
export WIFI_PASSWORD=$USER_INPUT_WIFI_PASSWORD
export WIFI_SSID=NETGEAR94
export MQTT_SERVER=192.168.0.196
export DEVICE_NAME=dispenser1
platformio run --target upload
