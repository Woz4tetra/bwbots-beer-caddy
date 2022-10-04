
BASE_DIR=$(realpath "$(dirname $0)")


echo "---"
echo "Installing dependencies via apt"
echo "---"

packages=(
joy
geometry2
vision-msgs
perception-pcl
pcl-msgs
apriltag-ros
costmap-converter
image-geometry
image-pipeline
image-common
laser-filters
robot-localization
gmapping
amcl
usb-cam
image-transport-plugins
rosbag-snapshot
twist-mux
)

package_list=""
for p in "${packages[@]}"; do
    package_list+="ros-noetic-$p "
done

sudo apt-get install -y $package_list libsdl-image1.2-dev libsdl-dev python3-pip
