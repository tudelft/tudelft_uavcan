## TU Delft UAVCAN modules
This repository contains the UAVCAN modules used by the TU Delft.


# Compile
 - ```git submodule update --init --recursive --force```
 - ```cd firmware```
 - Compile message definitions and protocol: DSDLC/generated:  ```make protocol```
 - firmware: ```make```


# Start CAN-UAV (configure CAN Nodes)
 - find the device name of your CAN-USB
 - sudo slcand -o -c -f -s8 /dev/ttyACM0 can0
 - sudo ifconfig can0 up
 - uavcan_gui_tool
 - dronecan
 
# CAN-tunnel for BLHeli devices
 - reboot devices ?
 - install ```pip3 install git+https://github.com/UAVCAN/pyuavcan@master```
 - or ```pip3 install pyuavcan_v0``` ??
 - ```./tools/test.py```

# BLHeli configure
 - BLHeliSuite32XL: https://github.com/bitdump/BLHeli/tree/master/BLHeli_32%20ARM#readme
