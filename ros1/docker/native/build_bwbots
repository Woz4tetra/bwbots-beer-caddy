#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

cd ${BASE_DIR}/../resources

docker-compose -f docker-compose.build_bwbots.yml up -d
docker logs -f build_bwbots
