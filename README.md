## TU Delft UAVCAN modules
This repository contains the UAVCAN modules used by the TU Delft.

# Compile
 - git submodules init & update
 - Compile message definitions and protocol: DSDLC/generated:  how ??
 - firmware: make


# Start CAN-UAV (configure CAN Nodes)
 - find the device name of your CAN-USB
 - sudo slcand -o -c -s8 /dev/ttyACM0 can0
 - sudo ifconfig can0 up
 - uavcan_gui_tool
 
# CAN-tunnel for BLHeli devices
 - reboot devices ?
 - ./tools/test.py


