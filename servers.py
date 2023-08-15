import socket
import time
import sys
import datetime
import struct
#import RPi.GPIO as GPIO
#define host ip: Rpi's IP
HOST_IP = "192.168.196.82"
HOST_PORT = 8880
print("Starting socket: TCP...")
#1.create socket object:socket=socket.socket(family,type)
socket_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#socket_tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0)
print("TCP server listen @ %s:%d!" %(HOST_IP, HOST_PORT) )
host_addr = (HOST_IP, HOST_PORT)
#2.bind socket to addr:socket.bind(address)
socket_tcp.bind(host_addr)
#3.listen connection request:socket.listen(backlog)
socket_tcp.listen(10)
#4.waite for client:connection,address=socket.accept()
socket_con, (client_ip, client_port) = socket_tcp.accept()
print("Connection accepted from %s." %client_ip)
#socket_con.send("Welcome to RPi TCP server!")
#5.handle
array_f=[1.1,2.2,3.3,4.4,5.5,6.6,7.7,8.8,9.9,10.1,11.1,]*6
data_array=struct.pack('<66f',*array_f)	
print("Receiving package...")


while True:
    try:
        data=socket_con.recv(1024)
        if len(data)>0:
            data_unpack=struct.unpack('<6f',data)
            print("unpack all","time:",time.time())
            
            #print("Received:%s"%data,"time:",time.time())
            
            socket_con.send(data_array)
            #time.sleep(1)
            continue
    except Exception:
            socket_tcp.close()
            sys.exit(1)
