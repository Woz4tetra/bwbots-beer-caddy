export WIFI_SSID=$1
read -s -p "Please enter the wifi password: " USER_INPUT_WIFI_PASSWORD
export WIFI_PASSWORD=$USER_INPUT_WIFI_PASSWORD
platformio run
