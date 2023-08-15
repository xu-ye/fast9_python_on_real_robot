import socket
import time
import sys
import datetime
import struct
#import RPi.GPIO as GPIO
#define host ip: Rpi's IP
HOST_IP = "192.168.196.82"
client_IP='192.168.196.213'
HOST_PORT = 8880
print("Starting socket: udp...")
#1.create socket object:socket=socket.socket(family,type)
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#udp_socket.setsockopt(socket.IPPROTO_udp, socket.udp_NODELAY, 0)
print("UDP server listen @ %s:%d!" %(HOST_IP, HOST_PORT) )
host_addr = (HOST_IP, HOST_PORT)
client_addr = (client_IP, HOST_PORT)
#2.bind socket to addr:socket.bind(address)
udp_socket.bind(host_addr)
#3.listen connection request:socket.listen(backlog)

#socket_con.send("Welcome to RPi udp server!")
#5.handle
array_f=[1.1,2.2,3.3,4.4,5.5,6.6,7.7,8.8,9.9,10.1,11.1,]*6
data_array=struct.pack('<66f',*array_f)	
print("Receiving package...")


while True:
    try:
        data=udp_socket.recvfrom(1024)
        if len(data)>0:
            data_unpack=struct.unpack('<6f',data[0])
            print("unpack all","time:",time.time())
            
            #print("Received:%s"%data,"time:",time.time())
            
            udp_socket.sendto(data_array,client_addr)
            #time.sleep(1)
            continue
    except Exception:
            udp_socket.close()
            sys.exit(1)


 