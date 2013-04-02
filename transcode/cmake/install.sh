#!/bin/bash

PATHS="/usr/lib/transcode /usr/local/lib/transcode $HOME/lib/transcode"
for D in $PATHS; do
    if [ -d $D ]; then
        echo -n "Do you want to install the plugins to $D ? [y/N] ";
        read choice
        if [ "$choice" = "y" ]; then
            echo -n "Copy files... "	
            SUDO=;
            if [ ! -w $D ]; then SUDO=sudo; echo " (need root permission, so I use sudo) "; fi
	    $SUDO cp -v --backup=numbered ./filter_stabilize.so ./filter_transform.so ./filter_deshake.so "$D/" && echo "done";
        fi
    fi
done