#!/usr/bin/env python3
import uavcan
import os, pty, serial
import threading
import time
import os
import sys
20
target_node_id = int(sys.argv[1])

master, slave = pty.openpty()
s_name = os.ttyname(slave)
not_done = False
req_more = False
print(s_name)

# Instantiating an instance of standard service type uavcan.protocol.GetNodeInfo
# Here we need the response data structure, which is why we use the factory Response()
node_info = uavcan.protocol.GetNodeInfo.Response()
node_info.name = 'org.uavcan.pyuavcan_demo'
node_info.software_version.major = 1
node_info.hardware_version.unique_id = b'12345' # Setting first 5 bytes; rest will be kept zero
# Fill other fields as necessary...

node = uavcan.make_node('can0',
                        node_id=110,          # Setting the node ID 143
                        node_info=node_info)  # Setting node info


#{0,0,0,0,0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
#send_cmd = b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0DBLHeli\xF4\x7D'
#print(len(send_cmd))

def do_publish(bytes = ''):
    global not_done
    msg = uavcan.tunnel.Call.Request(buffer=bytes)
    # The priority argument can be omitted, in which case default will be used
    #node.broadcast(msg, priority=uavcan.TRANSFER_PRIORITY_LOWEST)
    #print('-> Transmit to node ', target_node_id)
    print('-> ', bytes, len(bytes))
    #print('')

    not_done = True
    node.request(msg, target_node_id, tunnel_call_callback,
             priority=uavcan.TRANSFER_PRIORITY_HIGHEST, timeout=0.5)
    while not_done and not req_more:
        time.sleep(0.0001)

#handle = node.periodic(0.2, do_publish)

def tunnel_call_callback(event):
    global not_done
    global req_more
    if event:
        #print('<- Response from node ', event.transfer.source_node_id)
        if len(event.response.buffer) > 0:
            print('<- ', bytes(event.response.buffer), len(event.response.buffer)) 
            os.write(master, bytes(event.response.buffer))
        else:
            print('<-  ', 0)
        if len(event.response.buffer) == 60:
            #print('<- REQ MORE')
            req_more = True
        else:
            req_more = False
            not_done = False
    else:
        print('<- Response timed out!')
        not_done = False
    print('')

threading.Thread(target=node.spin, daemon=True).start()
while True:
    if req_more:
        req_more = False
        do_publish()
    else:
        msg = os.read(master, 60)
        if len(msg) > 0:
            os.write(master, msg)
            do_publish(msg)
    time.sleep(0.0001)