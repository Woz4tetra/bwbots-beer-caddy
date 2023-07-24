# Pull docker image

- Run this script on the Jetson: `~/bwbots-beer-caddy/docker/jetson/pull_container`

# Build docker container from scratch

Login into the Jetson, then run

- `cd ~/bwbots-beer-caddy/docker/jetson`

Run this once on the host:

- `sudo ./install_docker_dependencies.sh`

Run this to build the docker container:
`sudo ./build_container.sh`

After the above command completes successfully, run this:

- `./clean_build_ros_ws.sh`

Push image:

- `./push_container.sh`

# Helpful links

- https://www.stereolabs.com/docs/docker/building-arm-container-on-x86/
- https://docs.docker.com/build/building/multi-platform/
- https://roboticseabass.com/2021/04/21/docker-and-ros/
- https://stackoverflow.com/questions/46032392/docker-compose-does-not-allow-to-use-local-images
- https://stackoverflow.com/questions/23935141/how-to-copy-docker-images-from-one-host-to-another-without-using-a-repository
