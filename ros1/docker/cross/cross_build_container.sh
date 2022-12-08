#!/bin/bash

cd ../resources

docker buildx use bwbots-builder

for arch in linux/amd64 linux/arm64  ; do 
    docker buildx build \
    --platform $arch \
    --output type=docker \
    -f ./Dockerfile \
    --tag bwbots:latest .
done
