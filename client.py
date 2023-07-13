import socket
import time
import sys
import datetime
import numpy as np
import random 
import struct

#RPi's IP
SERVER_IP = "10.13.23.239"
SERVER_PORT = 8881
a = [1,2,3,4]
b=[1.2,2.3,3.4,4.5,5.6,6.6,1.2,2.3,3.4,4.5,5.6,6.6,1.2,2.3,3.4,4.5,5.6,6.6,1.2,2.3,3.4,4.5,5.6,]*7
c=[]

b1=bytearray(a)
for i in range(5):
    print(struct.pack('<f',b[i]))
    c.append(struct.pack('<f',b[i]))
#a_t = struct.pack('<f',a)
data1=struct.pack('<161f',*b)
#print(data1)
data=struct.unpack('<161f',data1)
#print(data)

#data=random.randint(0,10,(6,3)).encode(encoding='utf_8', errors='strict')
data=np.zeros((6,20))
print("Starting socket: TCP...")
server_addr = (SERVER_IP, SERVER_PORT)
socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:
    try:
        print("Connecting to server @ %s:%d..." %(SERVER_IP, SERVER_PORT))
        socket_tcp.connect(server_addr)
        break
    except Exception:
        print("Can't connect to server,try it latter!")
        time.sleep(1)
        continue
#print("Please input gogo or stop to turn on/off the motor!")
socket_tcp.send(data1)
socket_tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0) 
while True:
    try:
        data = socket_tcp.recv(1024)
        if len(data)>0:
            data_unpack=struct.unpack('<14f',data)
            #print("unpack all",data_unpack)
            
            #data1=list(bytearray(data))
            #if len(data1)==48:
                #data_receive=[struct.unpack('<f',data[4*i:4*i+4]) for i in range(12)]
                #print("unpack: ",data_receive)
            #data_array=data.split()
            #print("Received: ", data)
            #command="gogo".encode(encoding='utf_8', errors='strict')
            time1=time.time()
            #print("send:",datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'))
            print("send:",time1)
            #data=np.zeros((6,3))
            #print(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'))  
            socket_tcp.send(data1)
            #time.sleep(1)
            continue
    except Exception:
        socket_tcp.close()
        socket_tcp=None
        sys.exit(1)