# -*- coding: utf-8 -*
import serial
import time
import struct
import binascii 

#ser = serial.Serial("/dev/ttyAMA0",1000000,parity='E',timeout=0.000005)
ser = serial.Serial("/dev/ttyAMA0",115200*16,parity='E',timeout=0.001)

def crc16_cal(datalist):
    test_crc=0xFFFF                 #预置1个16位的寄存器为十六进制FFFF（即全为1），称此寄存器为CRC寄存器；
    poly=0xa001
    # poly=0x8005
    numl=len(datalist)
    for num in range(numl):
        data=datalist[num]
        test_crc=(data&0xFF)^test_crc   #把第一个8位二进制数据（既通讯信息帧的第一个字节）与16位的CRC寄存器的低8位相异或，把结果放于CRC寄存器，高八位数据不变；
        
        #右移动
        for bit in range(8):
            if(test_crc&0x1)!=0:
                test_crc>>=1
                test_crc^=poly
            else:
                test_crc>>=1
    #print(hex(test_crc))
    return test_crc


if not ser.isOpen():
    print("open failed")
else:
    print("open success: ")
    print(ser)
    
array_action=[1,2,3,4.5,0.4,0.1]*3
array_action_array=struct.pack('<18f',*array_action)
crc=crc16_cal(array_action_array)
crc1=crc.to_bytes(2,'big')
start='be'.encode()
int.from_bytes(crc1, byteorder='big', signed=True)
end=b'\n'
action_array_crc=array_action_array+crc1+end

#array_action_array=array_action_array+a
#ser.write(b'Hello, world!\n')
#ser.flushInput()
#ser.flushInput()
try:
    while True:
        count = ser.inWaiting()
        if count >= 108:
            #data1=ser.readline()
            #data = ser.readline()
            data1 = ser.read(108)
            print("count",count,time.time())
            #for line in data1:
            #data_unpack=data1.decode('utf-8')
            if count>=108:
                data_unpack=struct.unpack('<27f',data1)
                print("unpack all","time:",time.time())
            #data = ser.readline().decode('utf-8').rstrip()
            #print('Received: ' + data)
            #print(data1.decode('utf-8'),time.time())
            #ser.flushInput()
            #recv = ser.read(count)
            #data = ser.readline()
            #recv1=recv.decode('utf-8')
            #print("recv: " + recv1)
            line_data='happy\n'
            #ser.write(b'34h\n') # 向串口发送数据
            #ser.write(line_data.encode('utf-8'))
            ser.write(action_array_crc)
            '''
            while 1:
                count = ser.inWaiting()
                if count >= 2:
                    data1 = ser.read(2)
                    data_decode=data1.decode()
                    if data_decode=='OK':
                        print("ok")
                        break
                    else:
                        ser.write(action_array_crc)
                        print("re")
             '''       
            
            
        #time.sleep(0.5) 
except KeyboardInterrupt:
    if ser != None:
        ser.close()
