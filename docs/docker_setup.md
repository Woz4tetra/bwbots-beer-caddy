
# Pull docker image

- Obtain `bwbots_ws.tar.gz`
- Upload to the Jetson in `/home/nvidia/Downloads`
- `cd ~/bwbots-beer-caddy/ros1/docker/native`
- `./import_container.sh /home/nvidia/Downloads/bwbots_ws.tar.gz`

# Build docker container from scratch

- `cd ~/bwbots-beer-caddy/ros1/docker/native`

Run this once on the host:
- `sudo install_docker_dependencies.sh`

Run this to build the docker container:
`sudo build_container.sh`

After the above command completes successfully, run this:
- `./post_build.sh`

Push image:
- `./export_container.sh`

# Helpful links
- https://www.stereolabs.com/docs/docker/building-arm-container-on-x86/
- https://docs.docker.com/build/building/multi-platform/
- https://roboticseabass.com/2021/04/21/docker-and-ros/
- https://stackoverflow.com/questions/46032392/docker-compose-does-not-allow-to-use-local-images
- https://stackoverflow.com/questions/23935141/how-to-copy-docker-images-from-one-host-to-another-without-using-a-repository
