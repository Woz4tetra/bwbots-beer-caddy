# export WIFI_SSID=$1
# read -s -p "Please enter the wifi password: " USER_INPUT_WIFI_PASSWORD
# export WIFI_PASSWORD=$USER_INPUT_WIFI_PASSWORD
export WIFI_SSID=NETGEAR94
export WIFI_PASSWORD=fourwordsalluppercase
export MGTT_SERVER=192.168.0.16
platformio run --target upload
