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

gnome-terminal --title="FLIR NODE" -- /bin/sh -c ". ./run_ros_nodes.sh ;$SHELL"

# Run the program
python lemanchot-dc.py
