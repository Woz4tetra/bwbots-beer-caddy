if [ -z $1 ]; then
    read -s -p "Please enter the wifi password: " USER_INPUT_WIFI_PASSWORD
else
    USER_INPUT_WIFI_PASSWORD=$1
fi
export WIFI_PASSWORD=$USER_INPUT_WIFI_PASSWORD
export WIFI_SSID=NETGEAR94
export MQTT_SERVER=192.168.0.196
export DEVICE_NAME=drum_dispenser
platformio run --target upload
