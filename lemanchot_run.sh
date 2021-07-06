#!/bin/bash

# Install banner package
sudo apt install -y banner

echo ""
banner LEMANCHOT
echo "-----------------------------------------"

echo "============== LeManchot-DC | ULAVAL ================================"
echo ">> LeManchot-DC is a data acquisition system designed for DJI drones"
echo "The system uses ROS capabilities to run multiple nodes for providing"
echo "access to the multi-modal sensory platform designed by us."
echo "---"
echo "Developed by Parham Nooralishahi - PMH!"
echo "Contact >>> parham.nooralishahi@gmail.com"
echo "Supervisor >>> Professor Xavier Maldague"
echo "Organization >>> Laval University"
echo "ULAVAL   >>> https://www.ulaval.ca"
echo "@Winter 2021"
echo "======================================================================"

chmod u+x "./run_ros_nodes.sh"

FLIR_NODE=
while [ -n "$1" ]; do # while loop starts
	case "$1" in
	-a) echo "FLIR NODE is going to be initialized!" && export FLIR_NODE="flir";; # Message for -a option
	*) echo "Option $1 not recognized" ;; # In case you typed a different option other than a,b,c
	esac
	shift
done

if [ -n "$FLIR_NODE" ] 
then
    echo "FLIR NODE is running ..."
    gnome-terminal --title="FLIR NODE" -- /bin/sh -c ". ./run_ros_nodes.sh ;$SHELL"
fi

# Run the program
python3 lemanchot-dc.py

# import sys

# print('Remove the python 2.7 distro from the library path!')
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
