BASE_DIR=$(realpath "$(dirname $0)")
docker build -t bwbots:latest ${BASE_DIR}
