#!/bin/bash

sudo rm -r /tmp/workstation_${ORGANIZATION}/sys
sudo rsync -az /tmp/workstation_${ORGANIZATION}/* /
sudo rm -r /tmp/*
