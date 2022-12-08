#!/bin/bash

cd ../resources
docker build -f ./Dockerfile -t bwbots:latest .
