#!/bin/bash

cd ./resources

docker buildx use bwbots-builder
docker buildx build --platform linux/amd64/v4,linux/arm64 -f ./Dockerfile -t bwbots:latest .
