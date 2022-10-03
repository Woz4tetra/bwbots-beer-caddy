#!/usr/bin/env bash
BASE_DIR=$(realpath "$(dirname $0)")

while IFS="" read -r p || [ -n "$p" ]
do
    printf '%s\n' "$p"

    if grep -Fq "$p" ~/.bashrc ; then
        echo "command already exist in .bashrc"    
    else
        echo "Appending ros setup scripts to ~/.bashrc"
        printf '%s\n' "$p" | sudo tee -a ~/.bashrc > /dev/null
        echo "re-open this terminal session for bashrc to go into affect"
    fi
done < ${BASE_DIR}/bashrc_append.txt

WS_DIR=${HOME}/ros_ws/src/
mkdir -p ${WS_DIR}
SOURCE_DIR=$(realpath ${BASE_DIR}/../../bwbots)
ln -s ${SOURCE_DIR} ${WS_DIR}

echo "re-open this terminal session for bashrc to go into affect"
