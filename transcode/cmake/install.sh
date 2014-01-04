#!/bin/bash

PATHS="/usr/lib/transcode /usr/local/lib/transcode $HOME/lib/transcode"
for D in $PATHS; do
    if [ -d "$D" ]; then
        echo -n "Do you want to install the plugins to $D ? [y/N] ";
        read choice
        if [ "$choice" = "y" ]; then
            echo  "Checking PATH variable: "
            if [ ! -w "$D" ]; then SUDO=sudo; fi
            ${SUDO} cp --backup=numbered ./filter_stabilize.so ./filter_transform.so ./filter_deshake.so "$D/"
        fi
    fi
done