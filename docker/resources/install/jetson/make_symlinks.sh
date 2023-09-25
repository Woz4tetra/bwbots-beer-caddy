#!/bin/bash

set -e

ln -s /opt/${ORGANIZATION}/${PROJECT_NAME} ${HOME}/${PROJECT_NAME}

sudo rm -r /usr/local/zed/settings
ln -s ln /opt/${ORGANIZATION}/${PROJECT_NAME}/bw_data/data/zed/resources /usr/local/zed/settings

sudo rm -r /usr/local/zed/resources
ln -s ln /opt/${ORGANIZATION}/${PROJECT_NAME}/bw_data/data/zed/resources /usr/local/zed/resources
