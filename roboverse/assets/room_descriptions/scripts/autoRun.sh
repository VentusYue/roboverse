#!/bin/bash
# File to launch gepetto-viewer, hpp-corbaserver and a script.

# Command: sh autoRun.sh

# In first terminal : viewer-server
xterm -e gepetto-viewer-server &
sleep 0.5s
# In first terminal : corbaserver
xterm -e hppcorbaserver &
sleep 0.5s
# In second terminal : python script
xterm -e python autoRun_room.py &
sleep 300s
killall xterm
