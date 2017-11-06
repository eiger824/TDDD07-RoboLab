#!/bin/bash

# -- Config --
echo "Reading configuration..."
source config

# -- Run Sync --
echo "Sync in progress..."
rsync -avz --delete ./robot_agent/bin/robot_agent rescue@$ROBOT_IP:/home/rescue/TDDD07/LabGroup$LAB_GROUP/
echo "Sync done!"