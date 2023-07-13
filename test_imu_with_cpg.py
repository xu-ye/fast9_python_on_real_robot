import json
from sys import path
path.append("../../")
import math
import numpy as np
import random
#import Servos
import sys
import os

import socket
import setproctitle
import numpy as np
from pathlib import Path
from threading import Thread, current_thread

#from stable_baselines.common.env_checker import check_env

import math

import json
from multiprocessing import Process
from multiprocessing import Process, Queue
#import queue

from threading import Timer

import time
import datetime
import platform
import struct
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

import socket
import time
import sys
import datetime
import numpy as np
import random 
import struct
import csv




def sim_angles_to_real(theta):
    # theta [6,3]  输出 np array [18]
    real_angles=np.zeros_like(theta).flatten()
    for i in range(6):
        if i<3:
            
            hip=180+theta[i,0]/math.pi*180 #向前为增加
            
            knee=180-theta[i,1]/math.pi*180
            ankle=62.69+theta[i,2]/math.pi*180
            
            real_angles[3+i*6+0]=hip
            real_angles[3+i*6+1]=knee
            real_angles[3+i*6+2]=ankle
            
        else:
            hip=180+theta[i,0]/math.pi*180 #向前为增加
            knee=180+theta[i,1]/math.pi*180
            ankle=62.69-theta[i,2]/math.pi*180
            
            real_angles[0+(i-3)*6+0]=hip
            real_angles[0+(i-3)*6+1]=knee
            real_angles[0+(i-3)*6+2]=ankle
        
    return real_angles

def real_angles_to_sim(real_angles):
    # theta [6,3]
    theta=np.zeros((6,3))
    for i in range(6):
        if i<3:
            theta[i,0]= (real_angles[3+i*6+0]-180)/180*math.pi
            theta[i,1]= (-real_angles[3+i*6+1]+180)/180*math.pi
            theta[i,2]= (real_angles[3+i*6+2]-62.69)/180*math.pi
            
            
            
        else:
            theta[i,0]= (real_angles[0+(i-3)*6+0]-180)/180*math.pi
            theta[i,1]= (real_angles[0+(i-3)*6+1]-180)/180*math.pi
            theta[i,2]= (-real_angles[0+(i-3)*6+2]+62.69)/180*math.pi
            
           
        
    return theta


def real_current_to_sim_torque(real_current):
    # theta [6,3]
    torque_sim=np.zeros((6,3))
    real_torque1=real_current.reshape(6,3)
    for i in range(6):
        if i<3:
            torque_sim[i,0]= real_current[3+i*6+0]*2.69/1000*1.82-0.2576
            torque_sim[i,1]= -(real_current[3+i*6+1]*2.69/1000*1.82-0.2576)
            torque_sim[i,2]= real_current[3+i*6+2]*2.69/1000*1.82-0.2576
            
            
            
        else:
            torque_sim[i,0]= real_current[0+(i-3)*6+0]*2.69/1000*1.82-0.2576
            torque_sim[i,1]= real_current[0+(i-3)*6+1]*2.69/1000*1.82-0.2576
            torque_sim[i,2]= -(real_current[0+(i-3)*6+2]*2.69/1000*1.82-0.2576)
            
           
        
    return torque_sim

def get_forward_pos(pos):
    
    
        theta_1=0+pos[0]
        theta_2=-(90-21.36-0)/180*3.1415926-pos[1]
        theta_3=(180-(62.69+0-21.36-15))/180*3.1415926-pos[2]
        
        T_01=np.array([[np.cos(theta_1),-np.sin(theta_1),0,0],[np.sin(theta_1),np.cos(theta_1),0 ,0],[0, 0 ,1, -33.33/1000],[0, 0, 0 ,1]])
        T_12=np.array([[np.cos(theta_2),-np.sin(theta_2),0,42.55/1000],[0,0,1,0],[-np.sin(theta_2),-np.cos(theta_2),0,0],[0,0,0,1]])
        T_23=np.array([[np.cos(theta_3),-np.sin(theta_3),0,64.25/1000],[np.sin(theta_3),np.cos(theta_3),0,0],[0,0,1,0],[0,0,0,1]])
        T_34=np.array([[1,0,0,135.4/1000],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        T_04=T_01@T_12@T_23@T_34
        end_pos=T_04[0:3,3]
        ouler_angles=rotationMatrixToEulerAngles(T_04[0:3,0:3])
        return end_pos,ouler_angles

def rotationMatrixToEulerAngles(R) :

        
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])
    
    
 

def read_servos_send(servos,data_read,cpg_index,q_imu,socket_tcp):
    if cpg_index>239:
        cpg_next=0
    else:
        cpg_next=cpg_index+1
    Timer(0.04,read_servos_send,args=(servos,data_read,cpg_next,q_imu,socket_tcp)).start()
    
        
    print("read servo:",time.time())
    agent_num=7
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    theta_ini_r=np.asarray(data_read['theta_ini'])
    ini_index_r=np.asarray(data_read['ini_index'])
    torque_r=np.asarray(data_read['torque_n'])
    end_pos_r=np.asarray(data_read['end_pos'])
    
    
    
    start_t=time.time()
    position_Read,flag=servos.read_all_positions() # np array 18
    current_Read=servos.read_all_current() # np array 18
    
    end_t=time.time()
    print("read time: ",end_t-start_t)
    
    theta_sim=real_angles_to_sim(position_Read)
    torque_sim=real_current_to_sim_torque(current_Read)
    
    sim_angles_to_real(theta_sim)
    ## imu data
    IMU_data=q_imu.get(True,10)
    while( not q_imu.empty()):
        IMU_data=q_imu.get(True,10)
    #IMU_data=np.array([0,0,0,0,0,0,])
    (roll ,pitch,yaw)=IMU_data[0:3]/180*math.pi # 绕着 xy z轴转动
    length_12=0.126
    length_13=0.266
    dz1=-length_12*math.tan(roll)
    dz2=-length_13*math.tan(roll)
    
    comtact_force_leg=np.zeros(agent_num-1)
    foot_z=np.zeros(agent_num-1)
    phase=phase_r[cpg_index,:]
    observation_temp=[]
    
    
    
    
    torque_n=torque_r[cpg_index]
    for agent_index in range(agent_num):

        if agent_index<6:
            robot_joint_positions_agenti=theta_sim[agent_index]
            robot_joint_Torque_agenti=torque_sim[agent_index]
            #robot_joint_Torque_agenti = torque_sim[3*agent_index:3*agent_index+3]
            #world_end_position=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True)[0]
            #zworld_end_ori=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True)[0]
            if agent_index>=3:
                end_pos,end_ori=get_forward_pos(-robot_joint_positions_agenti)
            else:
                end_pos,end_ori=get_forward_pos(robot_joint_positions_agenti)
            
            foot_z[agent_index]=end_pos[2]
            if agent_index%3==1:
                foot_z[agent_index]=end_pos[2]+dz1
                
            if agent_index%3==2:
                foot_z[agent_index]=end_pos[2]+dz2
        
            # 误差的error
            
            torque_target=torque_n[agent_index,:]
            torque_error=torque_target-robot_joint_Torque_agenti
            
            expeted_end_pos=end_pos_r[cpg_index,agent_index,:]
            end_pos_error=expeted_end_pos-end_pos



            # 相位信息
            
            phase_continus=cpg_r[agent_index,2:4,cpg_index]
            
            ## 不全知信息
            contact_force_v=0
            contact_force_diff=0
            contact_force_l=0
            contact_point_num=0
            height_now_n_next=np.array([0,0])
            step_width=0
            
            
            observation_agenti = np.append(robot_joint_positions_agenti, robot_joint_Torque_agenti)
            observation_agenti = np.append(observation_agenti,contact_force_v/10)
            observation_agenti = np.append(observation_agenti,contact_force_diff/10)
            observation_agenti = np.append(observation_agenti,contact_force_l/10)
            observation_agenti = np.append(observation_agenti,contact_point_num)
            observation_agenti = np.append(observation_agenti,height_now_n_next*10)
            observation_agenti = np.append(observation_agenti,step_width*10)
            observation_agenti = np.append(observation_agenti,end_pos*10)
            observation_agenti = np.append(observation_agenti,end_pos_error[0:2]*100)
            observation_agenti = np.append(observation_agenti,phase_continus)
            observation_agenti = np.append(observation_agenti,torque_error)
            observation_temp.append(observation_agenti)
                
        else:
            IMU=IMU_data[0:3]*1/180*math.pi
            print("IMU",IMU_data[0:3])
            contact_force_flag=comtact_force_leg*phase*-1
            relative_foot_z=foot_z-foot_z[0]
            observation_agenti = np.append(IMU, contact_force_flag/10)
            observation_agenti = np.append(observation_agenti,relative_foot_z*10)
            observation_agenti = np.append(observation_agenti,IMU)
            observation_agenti = np.append(observation_agenti,np.zeros(2))
            observation_agenti = np.append(observation_agenti,np.zeros(3))
           
            observation_temp.append(observation_agenti)
            
    
    
    #return observation_temp
    observation = np.array(np.vstack(observation_temp),dtype=np.float32).flatten()
    data_obs=struct.pack('<161f',*observation)
    
    
    socket_tcp.send(data_obs)
    print("send time:",time.time())
            
            
    #Timer(0.040,read_servos,args=(servos,data_read,cpg_index,q_imu,)).start()


def read_servos_only(servos,data_read,cpg_index,q_imu,socket_tcp,step):
    global imu_init
    #global position_Read
    #Timer(0.04,read_servos,args=(servos,data_read,cpg_index,q_imu,socket_tcp)).start()
    print("read servo:",time.time())
    agent_num=7
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    theta_ini_r=np.asarray(data_read['theta_ini'])
    ini_index_r=np.asarray(data_read['ini_index'])
    torque_r=np.asarray(data_read['torque_n'])
    end_pos_r=np.asarray(data_read['end_pos'])
    
    
    start_t=time.time()
    position_Read,flag=servos.read_all_positions() # np array 18
    while(flag==0):
        position_Read,flag=servos.read_all_positions()
    current_Read,flag=servos.read_all_current() # np array 18
    while(flag==0):
        current_Read,flag=servos.read_all_current() # np array 18
        
    
    end_t=time.time()
    print("read time: ",end_t-start_t)
    
    theta_sim=real_angles_to_sim(position_Read)
    torque_sim=real_current_to_sim_torque(current_Read)
    
    sim_angles_to_real(theta_sim)
    ## imu data
    IMU_data=q_imu.get(True,10)
    while( not q_imu.empty()):
        IMU_data=q_imu.get(True,10)
    if step==0:
        imu_init=IMU_data
    else:
        IMU_data[0:3]=IMU_data[0:3]-imu_init[0:3]
        
    #IMU_data=np.array([0,0,0,0,0,0,])
    (roll ,pitch,yaw)=IMU_data[0:3]/180*math.pi # 绕着 xy z轴转动
    length_12=0.126
    length_13=0.266
    dz1=-length_12*math.tan(roll)
    dz2=-length_13*math.tan(roll)
    
    comtact_force_leg=np.zeros(agent_num-1)
    foot_z=np.zeros(agent_num-1)
    phase=phase_r[cpg_index,:]
    observation_temp=[]
    
    
    
    
    torque_n=torque_r[cpg_index]
    for agent_index in range(agent_num):

        if agent_index<6:
            robot_joint_positions_agenti=theta_sim[agent_index]
            robot_joint_Torque_agenti=torque_sim[agent_index]
            #robot_joint_Torque_agenti = torque_sim[3*agent_index:3*agent_index+3]
            #world_end_position=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True)[0]
            #zworld_end_ori=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True)[0]
            if agent_index>=3:
                end_pos,end_ori=get_forward_pos(-robot_joint_positions_agenti)
            else:
                end_pos,end_ori=get_forward_pos(robot_joint_positions_agenti)
            
            foot_z[agent_index]=end_pos[2]
            if agent_index%3==1:
                foot_z[agent_index]=end_pos[2]+dz1
                
            if agent_index%3==2:
                foot_z[agent_index]=end_pos[2]+dz2
        
            # 误差的error
            
            torque_target=torque_n[agent_index,:]
            torque_error=torque_target-robot_joint_Torque_agenti
            
            expeted_end_pos=end_pos_r[cpg_index,agent_index,:]
            end_pos_error=expeted_end_pos-end_pos



            # 相位信息
            
            phase_continus=cpg_r[agent_index,2:4,cpg_index]
            
            ## 不全知信息
            contact_force_v=0
            contact_force_diff=0
            contact_force_l=0
            contact_point_num=0
            height_now_n_next=np.array([0,0])
            step_width=0
            print("pos error",end_pos_error,"torque_error",torque_error,"torque_target",torque_target,"\n")
            
            
            observation_agenti = np.append(robot_joint_positions_agenti, robot_joint_Torque_agenti)
            observation_agenti = np.append(observation_agenti,contact_force_v/10)
            observation_agenti = np.append(observation_agenti,contact_force_diff/10)
            observation_agenti = np.append(observation_agenti,contact_force_l/10)
            observation_agenti = np.append(observation_agenti,contact_point_num)
            observation_agenti = np.append(observation_agenti,height_now_n_next*10)
            observation_agenti = np.append(observation_agenti,step_width*10)
            observation_agenti = np.append(observation_agenti,end_pos*10)
            observation_agenti = np.append(observation_agenti,end_pos_error[0:2]*100)
            observation_agenti = np.append(observation_agenti,phase_continus)
            observation_agenti = np.append(observation_agenti,torque_error)
            observation_temp.append(observation_agenti)
                
        else:
            IMU=IMU_data[0:3]*1/180*math.pi
            print("IMU",IMU_data[0:3])
            contact_force_flag=comtact_force_leg*phase*-1
            relative_foot_z=foot_z-foot_z[0]
            observation_agenti = np.append(IMU, contact_force_flag/10)
            observation_agenti = np.append(observation_agenti,relative_foot_z*10)
            observation_agenti = np.append(observation_agenti,IMU)
            observation_agenti = np.append(observation_agenti,np.zeros(2))
            observation_agenti = np.append(observation_agenti,np.zeros(3))
           
            observation_temp.append(observation_agenti)
            
    
    
    #return observation_temp
    observation = np.array(np.vstack(observation_temp),dtype=np.float32).flatten()
    #data_obs=struct.pack('<161f',*observation)
    
    
    #socket_tcp.send(data_obs)
    #print("send time:",time.time())
    return observation,position_Read


def readConfig(device):
    """
    读取配置信息示例    Example of reading configuration information
    :param device: 设备模型 Device model
    :return:
    """
    tVals = device.readReg(0x02,3)  #读取数据内容、回传速率、通讯速率   Read data content, return rate, communication rate
    if (len(tVals)>0):
        print("返回结果：" + str(tVals))
    else:
        print("无返回")
    tVals = device.readReg(0x23,2)  #读取安装方向、算法  Read the installation direction and algorithm
    if (len(tVals)>0):
        print("返回结果：" + str(tVals))
    else:
        print("无返回")

def setConfig(device):
    """
    设置配置信息示例    Example setting configuration information
    :param device: 设备模型 Device model
    :return:
    """
    device.unlock()                # 解锁 unlock
    time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
    device.writeReg(0x03, 6)       # 设置回传速率为10HZ    Set the transmission back rate to 10HZ
    time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
    device.writeReg(0x23, 0)       # 设置安装方向:水平、垂直   Set the installation direction: horizontal and vertical
    time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
    device.writeReg(0x24, 0)       # 设置安装方向:九轴、六轴   Set the installation direction: nine axis, six axis
    time.sleep(0.1)                # 休眠100毫秒    Sleep 100ms
    device.save()                  # 保存 Save

def AccelerationCalibration(device):
    """
    加计校准    Acceleration calibration
    :param device: 设备模型 Device model
    :return:
    """
    device.AccelerationCalibration()                 # Acceleration calibration
    print("加计校准结束")

def FiledCalibration(device):
    """
    磁场校准    Magnetic field calibration
    :param device: 设备模型 Device model
    :return:
    """
    device.BeginFiledCalibration()                   # 开始磁场校准   Starting field calibration
    if input("请分别绕XYZ轴慢速转动一圈，三轴转圈完成后，结束校准（Y/N)？").lower()=="y":
        device.EndFiledCalibration()                 # 结束磁场校准   End field calibration
        print("结束磁场校准")

def onUpdate(deviceModel):
    """
    数据更新事件  Data update event
    :param deviceModel: 设备模型    Device model
    :return:
    """
    global IMU_data
    '''
    print("芯片时间:" + str(deviceModel.getDeviceData("Chiptime"))
         , " 温度:" + str(deviceModel.getDeviceData("temperature"))
         , " 加速度：" + str(deviceModel.getDeviceData("accX")) +","+  str(deviceModel.getDeviceData("accY")) +","+ str(deviceModel.getDeviceData("accZ"))
         ,  " 角速度:" + str(deviceModel.getDeviceData("gyroX")) +","+ str(deviceModel.getDeviceData("gyroY")) +","+ str(deviceModel.getDeviceData("gyroZ"))
         , " 角度:" + str(deviceModel.getDeviceData("angleX")) +","+ str(deviceModel.getDeviceData("angleY")) +","+ str(deviceModel.getDeviceData("angleZ"))
        , " 磁场:" + str(deviceModel.getDeviceData("magX")) +","+ str(deviceModel.getDeviceData("magY"))+","+ str(deviceModel.getDeviceData("magZ"))
        , " 经度:" + str(deviceModel.getDeviceData("lon")) + " 纬度:" + str(deviceModel.getDeviceData("lat"))
        , " 航向角:" + str(deviceModel.getDeviceData("Yaw")) + " 地速:" + str(deviceModel.getDeviceData("Speed"))
         , " 四元素:" + str(deviceModel.getDeviceData("q1")) + "," + str(deviceModel.getDeviceData("q2")) + "," + str(deviceModel.getDeviceData("q3"))+ "," + str(deviceModel.getDeviceData("q4"))
          )
    '''
    IMU_data=np.array([deviceModel.getDeviceData("angleX"),deviceModel.getDeviceData("angleY"),deviceModel.getDeviceData("angleZ"),deviceModel.getDeviceData("gyroX"),
                       deviceModel.getDeviceData("gyroY"),deviceModel.getDeviceData("gyroZ"),deviceModel.getDeviceData("accX"),deviceModel.getDeviceData("accY"),deviceModel.getDeviceData("accZ")])
    #q_imu_1.append(q_imu)
    q_imu_1.put(IMU_data)
    if (0):    #记录数据    Record data
        Tempstr = " " + str(deviceModel.getDeviceData("Chiptime"))
        Tempstr += "\t"+str(deviceModel.getDeviceData("accX")) + "\t"+str(deviceModel.getDeviceData("accY"))+"\t"+ str(deviceModel.getDeviceData("accZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("gyroX")) +"\t"+ str(deviceModel.getDeviceData("gyroY")) +"\t"+ str(deviceModel.getDeviceData("gyroZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("angleX")) +"\t" + str(deviceModel.getDeviceData("angleY")) +"\t"+ str(deviceModel.getDeviceData("angleZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("temperature"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("magX")) +"\t" + str(deviceModel.getDeviceData("magY")) +"\t"+ str(deviceModel.getDeviceData("magZ"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("lon")) + "\t" + str(deviceModel.getDeviceData("lat"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("Yaw")) + "\t" + str(deviceModel.getDeviceData("Speed"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("q1")) + "\t" + str(deviceModel.getDeviceData("q2"))
        Tempstr += "\t" + str(deviceModel.getDeviceData("q3")) + "\t" + str(deviceModel.getDeviceData("q4"))
        Tempstr += "\r\n"
        #_writeF.write(Tempstr)

def startRecord():
    """
    开始记录数据  Start recording data
    :return:
    """
    global _writeF
    global _IsWriteF
    _writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S')) + ".txt", "w")    #新建一个文件
    _IsWriteF = True                                                                        #标记写入标识
    Tempstr = "Chiptime"
    Tempstr +=  "\tax(g)\tay(g)\taz(g)"
    Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    Tempstr += "\tT(°)"
    Tempstr += "\tmagx\tmagy\tmagz"
    Tempstr += "\tlon\tlat"
    Tempstr += "\tYaw\tSpeed"
    Tempstr += "\tq1\tq2\tq3\tq4"
    Tempstr += "\r\n"
    _writeF.write(Tempstr)
    print("开始记录数据")

def endRecord():
    """
    结束记录数据  End record data
    :return:
    """
    global _writeF
    global _IsWriteF
    _IsWriteF = False             # 标记不可写入标识    Tag cannot write the identity
    _writeF.close()               #关闭文件 Close file
    print("结束记录数据")  


def read_imu(q_imu):
     #init servos
    global  q_imu_1
    q_imu_1=q_imu
    device = deviceModel.DeviceModel(
        "我的JY901",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )

    if (platform.system().lower() == 'linux'):
        device.serialConfig.portName = "/dev/ttyUSB0"   #设置串口   Set serial port
    else:
        device.serialConfig.portName = "COM39"          #设置串口   Set serial port
    device.serialConfig.baud = 230400                     #设置波特率  Set baud rate
    device.ADDR=0x50
    device.openDevice()                                 #打开串口   Open serial port
    #setConfig()
    
    readConfig(device)                                  #读取配置信息 Read configuration information
    
    device.dataProcessor.onVarChanged.append(onUpdate)  #数据更新事件 Data update event
    #q_imu.put(IMU_data)
    

def receive(socket_tcp):
    while True:
        
        data = socket_tcp.recv(56)
        if len(data)>0:
            data_unpack=struct.unpack('<14f',data)
                
            time1=time.time()
                #print("send:",datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'))
            print("receive:",time1,"action: ",data_unpack,"len",len(data))
                #data=np.zeros((6,3))
                #print(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'))  
                #socket_tcp.send(data1)
                #time.sleep(1)
            continue
        

def cal_angles(action,cpg_index,data_read,position_Read,q_act):
    
    coef_cf=0.2
    
    
    
    theta_r = np.asarray(data_read['theta'])
    torque_r=np.asarray(data_read['torque_n'])
    
    
    
    
    # 输入actions  (6,2)   
    d_theta=np.zeros((6,3))
    #print("action: ",action)
    balance_theta=np.zeros(2)
    balance_theta[0]=action[6][0]
    balance_theta[1]=action[6][1]

        #默认输出归一化后到-1 到1
    for j in range(6):
        if action[j][1]<0 and j<3:
            action[j][1]=action[j][1]*1
        if action[j][1]>0 and j>3:
            action[j][1]=action[j][1]*1
        d_theta[j,0]=action[j][0]*0.3
        d_theta[j,1] =action[j][1]*coef_cf
        d_theta[j,2] =action[j][1]*coef_cf
        if j<3:
            d_theta[j,1] +=balance_theta[0]*coef_cf
        else:
            d_theta[j,1] +=balance_theta[1]*coef_cf  
    
    
    

    theta1=np.squeeze(theta_r[cpg_index,:,:])
    #theta_sim=d_theta+theta1*1   
    theta_sim=0+theta1*1   
    limit=0.75
    theta_sim=np.clip(theta_sim,-limit,limit)
    d_theta_real=sim_angles_to_real(d_theta+theta1)
    theta_real=sim_angles_to_real(theta_sim)
    
    inter_num=5
    print("theta_real",theta_real,"action_theta",d_theta_real,"cpg_index",cpg_index,"\n")
    position_Read=np.array(position_Read)
    for index in range(inter_num):
        theta_app=position_Read+(theta_real-position_Read)/inter_num*(index+1)
        print("theta1",theta_app,"\n")
        q_act.put(theta_app)
    return theta_real
    

def actuator_control(q_act,servos):
    #Timer(0.0100,actuator_control,args=(q_act,servos)).start()
    while 1:
        theta_real=q_act.get(True)
        #position_Read=servos.read_all_positions()
        #print(" actuator position ",position_Read)
        servos.write_all_positions_angles(theta_real)
        #print("theta :",theta_real[0:6],time.time())
        time.sleep(0.01990)
                 
                





import numpy as np

from Servos import *

if __name__ == '__main__':
    #global IMU_data 
    
    
    #IMU_data =np.array([1,1,1,0,0,0,1,1,1,])
    with open('force_real7.json', 'r') as f:
    
        data_read = json.load(f)
        force_r = np.asarray(data_read['contact_force'])
        phase_r = np.asarray(data_read['phase'])
        cpg_r = np.asarray(data_read['cpg'])
        theta_r = np.asarray(data_read['theta'])
        theta_ini_r=np.asarray(data_read['theta_ini'])
        ini_index_r=np.asarray(data_read['ini_index'])
        #torque_n=np.asarray(data_read['torque_n'])


    step= 0
    cpg_index=ini_index_r[0]
    
    #q_imu=queue.LifoQueue()
    q_imu= Queue()
    q_act=Queue()
    # socket
    
    
    
   
    
    Read_IMU = Process(target=read_imu,args=(q_imu,) )
    Read_IMU.start()
    time.sleep(2)
    
    servos=Servos()
    
    
    
    
    #servos.light_LED()
    #goal_position=2048*np.ones(1,18)
    servos.read_voltage(1
                        )
    servos.set_position_control()


    goal_position=np.array([180,204,85,180,204,85])
    position_Read=servos.read_position_loop()
    #read_servos(servos,data_read,cpg_index,q_imu,socket_tcp)
    
    print("Press any key to enable torque ! (or press ESC to no enable!)")
    if getch() != chr(0x1b):
    
    
        DXLn_ID=[0,1,2,3,4,5]
        servos.enable_torque(DXLn_ID)
        DXLn_ID=[6,7,8,9,10,11]
        servos.enable_torque(DXLn_ID)
        DXLn_ID=[12,13,14,15,16,17]
        servos.enable_torque(DXLn_ID)
    
    theta_sim=theta_r[cpg_index,:,:]
    angles_real=sim_angles_to_real(theta_sim)
    print("Press any key to initialize ! (or press ESC to no enable!)")
    if getch() != chr(0x1b):
        servos.Robot_initialize(angles_real)
    
    
    IMU_data_all=[]
    IMU_data=q_imu.get(True,10)
    IMU_data_all.append(IMU_data)
    data=IMU_data.tolist()
    #writer.writerow(data)
    IMU_data_all.append(data)
            
    print(" roll pitch yaw",IMU_data[0:3])
    print(" vel ",IMU_data[3:])
    print(" acc",IMU_data[6:])
    IMU_data=q_imu.get(True,10)
    
    data=IMU_data.tolist()
    #writer.writerow(data)
    IMU_data_all.append(data)
    
            
    print(" roll pitch yaw",IMU_data[0:3])
    print(" vel ",IMU_data[3:])
    print(" acc",IMU_data[6:])
    
    
    

    for step in range(240*4):
        start_time_t=time.time()
        
        while cpg_index >=240:
            cpg_index=cpg_index-240
            

        IMU_data=q_imu.get(True,10)
        while( not q_imu.empty()):
            IMU_data=q_imu.get(True,10)
            
        #print(" roll pitch yaw",IMU_data[0:3])
        #print(" vel ",IMU_data[3:])
        #print(" acc",IMU_data[6:])
        
        data=IMU_data.tolist()
        #writer.writerow(data)
        IMU_data_all.append(data)


        theta_sim=theta_r[cpg_index,:,:]
        angles_real=sim_angles_to_real(theta_sim)
        servos.write_all_positions_angles(angles_real)
        
        #position_goal = np.trunc(angles_real / 360 * 4096)  # tick
        #print("Press any key to continue! (or press ESC to escape)")

        #if getch() != chr(0x1b):
            #servos.write_some_positions(angles_real[0:6],DXLn_ID)
        #servos.write_all_positions_angles(angles_real)
            
        end_time_t=time.time()
        print("last time",end_time_t-start_time_t,"step",step)
        
        cpg_index=cpg_index+2
    
    file = open('test_ori_all1.csv', 'w')
    writer = csv.writer(file)
    head=['roll','pitch','yaw','x_v','y_v','z_v','x_acc','y_acc','z_acc',]
    writer.writerow(head)
    writer.writerows(IMU_data_all)
    file.close()
    
    
    
        
