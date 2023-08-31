#coding=GBK"
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
import numpy as np

from Servos import *

from set_imu import *
from utils import *
from reflex_related import *

import pybullet as p
import pybullet_data as pd
import math
import numpy as np
import random
path.append("./hexapod_real-main/Hexapod_Real/envs/")
from reflex_related import *
from pybullet_functions import *
from distill_model.action_net import *
import torch

import serial
import time
import struct






def read_servos_only1(Position_Read,relative_z,cpg_index,step,IMU_data,coef_real,phase_now,cpg_now):
    
    # 输出应该是 仿真环境下的机器人状态
    # imu 输入的应该是角度 单位为度数
    agent_num=6
    start_t0=time.time()
    
    #phase_r = np.asarray(data_read['phase'])
    #cpg_r = np.asarray(data_read['cpg'])
    #print("read data time: ",(time.time()-start_t0)*1000)

    start_t1=time.time()
    theta_sim=real_angles_to_sim(Position_Read)
    phase=phase_now
    observation_temp=[]
    #print("cal time: ",(time.time()-start_t1)*1000)
    
    start_t111=time.time()
    for agent_index in range(agent_num):
        #start_t0=time.time()
        
        robot_joint_positions_agenti=theta_sim[agent_index]     
        relative_foot_z=np.array([relative_z[agent_index]])
        phase_continus=cpg_now[agent_index,2:4]
        #start_t0=time.time()
        observation_agenti=np.concatenate((robot_joint_positions_agenti,IMU_data,relative_foot_z,phase_continus,np.array([phase[agent_index]]),np.array([coef_real[agent_index]]),),)
        #print("concatenate time: ",(time.time()-start_t0)*1000)
            
        
            #observation_agenti = np.append(robot_joint_positions_agenti, imu)
            #observation_agenti = np.append(observation_agenti,relative_foot_z)
            #observation_agenti = np.append(observation_agenti,phase_continus)
            #observation_agenti = np.append(observation_agenti,phase[agent_index])
            #observation_agenti = np.append(observation_agenti,coef_real[agent_index])
        observation_temp.append(observation_agenti)
       
    
    observation = np.array(np.vstack(observation_temp)).flatten()
    #print("loop time: ",(time.time()-start_t111)*1000)
    return observation






def read_servos_only(servos,data_read,cpg_index,step,imu,coef_real):
    
    # 输出应该是 仿真环境下的机器人状态
    # imu 输入的应该是角度 单位为度数
    agent_num=6
    start_t0=time.time()
    
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    print("read data time: ",(time.time()-start_t0)*1000)
    start_t0=time.time()
    position_Read=servos.read_all_positions() # np array 18
    
    end_t=time.time()
    print("read time: ",(end_t-start_t0)*1000)
    start_t1=time.time()
    theta_sim=real_angles_to_sim(position_Read)
   
    (roll ,pitch,yaw)=imu[0:3]/180*math.pi # 绕着 xy z轴转动
    length_12=0.126
    length_13=0.266
    dz1=-length_12*math.tan(roll)
    dz2=-length_13*math.tan(roll)
    foot_z=np.zeros(6)
    phase=phase_r[cpg_index,:]
    observation_temp=[]
    print("cal time: ",(time.time()-start_t1)*1000)
    
    start_t111=time.time()
    for agent_index in range(agent_num):
        start_t0=time.time()
        
        robot_joint_positions_agenti=theta_sim[agent_index]
        if agent_index>=3:
            end_pos,end_ori=get_forward_pos(-robot_joint_positions_agenti)
        else:
            end_pos,end_ori=get_forward_pos(robot_joint_positions_agenti)
            
        foot_z[agent_index]=end_pos[2]
        if agent_index%3==1:
            foot_z[agent_index]=end_pos[2]+dz1            
        if agent_index%3==2:
            foot_z[agent_index]=end_pos[2]+dz2
            
        relative_foot_z=np.array([foot_z[agent_index]-foot_z[0]])
        print("forward pos time: ",(time.time()-start_t0)*1000)
            # 相位信息
            
        phase_continus=cpg_r[agent_index,2:4,cpg_index]
        start_t0=time.time()
        observation_agenti=np.concatenate((robot_joint_positions_agenti,imu,relative_foot_z,phase_continus,np.array([phase[agent_index]]),np.array([coef_real[agent_index]]),),)
        print("concatenate time: ",(time.time()-start_t0)*1000)
            
        start_t0=time.time()
            #observation_agenti = np.append(robot_joint_positions_agenti, imu)
            #observation_agenti = np.append(observation_agenti,relative_foot_z)
            #observation_agenti = np.append(observation_agenti,phase_continus)
            #observation_agenti = np.append(observation_agenti,phase[agent_index])
            #observation_agenti = np.append(observation_agenti,coef_real[agent_index])
        observation_temp.append(observation_agenti)
        print("append time: ",(time.time()-start_t0)*1000)
    
    observation = np.array(np.vstack(observation_temp)).flatten()
    print("loop time: ",(time.time()-start_t111)*1000)
    return observation,position_Read




     
def set_pybullet():
    ## 加载servo_client
    servo_client=p.connect(p.DIRECT)
        #servo_client=p.connect(p.GUI)
    p.setGravity(0, 0, -9.8,physicsClientId=servo_client)  # 设置重力值
    p.setAdditionalSearchPath(pd.getDataPath(),physicsClientId=servo_client)  # 设置pybullet_data的文件路径

        # 加载地面
    floor_servo = p.loadURDF("plane.urdf",physicsClientId=servo_client)
    box_servo=[]
    box_servo.append(floor_servo)

    startPos_servo=[0,0,1]
        # 加载urdf文件
    robot_servo = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos_servo,physicsClientId=servo_client,useFixedBase=True,)
    dt=1/1000
    p.setTimeStep(dt,physicsClientId=servo_client)        
        
def servo_pybullet(q_servo_obs_now,q_servo_obs_next,q_pos_read):
    servo_client=p.connect(p.DIRECT)
        #servo_client=p.connect(p.GUI)
    p.setGravity(0, 0, -9.8,physicsClientId=servo_client)  # 设置重力值
    p.setAdditionalSearchPath(pd.getDataPath(),physicsClientId=servo_client)  # 设置pybullet_data的文件路径

        # 加载地面
    floor_servo = p.loadURDF("plane.urdf",physicsClientId=servo_client)
    box_servo=[]
    box_servo.append(floor_servo)

    startPos_servo=[0,0,1]
        # 加载urdf文件
    robot_servo = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos_servo,physicsClientId=servo_client,useFixedBase=True,)
    dt=1/1000
    p.setTimeStep(dt,physicsClientId=servo_client)    
    
    while 1:
        if q_servo_obs_now.empty()==False:
            obs_now=q_servo_obs_now.get(True,10)
            #obs_now1=obs_now.flatten()
            for actuator_index in range(18):
                p.resetJointState(robot_servo,actuator_index,obs_now[actuator_index])
            #position_servo=get_position(robot_servo,servo_client)
            print("reset  position")
            #q_pos_read.put(position_servo)
        if q_servo_obs_next.empty()==False:
            
            obs_next=q_servo_obs_next.get(True,10)
            timek=time.time()
            set_position_multi(obs_next,robot_servo,servo_client)
            position_servo=get_position(robot_servo,servo_client)
            print("pybullet",(time.time()-timek)*1000)
            q_pos_read.put(position_servo)
  
    
    
            
       



def load_and_actor(model1_dir,q_obs,q_act):
    obs_dim_t = 11

    act_dim = 1
    action_type = 'Continuous'
    num_agents=6
    n_embd=64
    device = torch.device("cpu")
    obs_dim=obs_dim_t*num_agents
    
    action_net1=ActionNet(obs_dim, n_embd, act_dim*num_agents,device)
    saved_model_1 = torch.load(model1_dir,map_location=torch.device('cpu'))
    action_net1.load_state_dict(saved_model_1)
    
    while 1:
        
        if q_obs.empty()==False:
            #print("q_obs empty  ",q_obs.empty())
            obs=q_obs.get(True,10)
            #print("obs",obs)
            obs_tensor=torch.tensor(obs).to(device)
            
            #while( not q_obs.empty()):
            #    obs=q_obs.get(True,10)
            #print("action net",action_net1)
            start_t=time.time()
            action=action_net1(obs_tensor)
            #action=action_net1(obs)
            #action=action_net1(obs)
            action_np=action.detach().numpy()
            #print("action",action_np)
            q_act.put(action_np)
            end_t=time.time()
            #pid = os.getpid()
            # 显示进程号
            
            print("network last time",(end_t-start_t)*1000)


              

def reflex_(ser,q_servo_obs_now,q_servo_obs_next,q_pos_read,q_obs1,q_act,num_T):
    

    #set_pybullet()
    
    
    with open('force_real17.json', 'r') as f:
    
        data_read_json = json.load(f)
        force_r = np.asarray(data_read_json['contact_force'])
        phase_r = np.asarray(data_read_json['phase'])
        cpg_r = np.asarray(data_read_json['cpg'])
        theta_r = np.asarray(data_read_json['theta'])
        theta_ini_r=np.asarray(data_read_json['theta_ini'])
        ini_index_r=np.asarray(data_read_json['ini_index'])
        torque_r=np.asarray(data_read_json['torque_n'])
        end_pos_r=np.asarray(data_read_json['end_pos'])
    
    
    
    #with open('pos_0_5_10_1.json', 'r') as f:
    with open('pos_20_17.json', 'r') as f:
        
        data_read = json.load(f)
        positions_tick = np.asarray(data_read['positions_tick'])
        current_pos_tick = np.asarray(data_read['current_pos_tick'])
        goal_pos_sim = np.asarray(data_read['goal_pos_sim'])
        phase = np.asarray(data_read['phase'])
        
    step= 0
    cpg_index=ini_index_r[0]
    
    

    
    flat_cpg_tick=current_pos_tick[step] #18
    

    # init vaiables
    positions=[]
    traj_error_buf=np.zeros((4,18))
    on_reflex=np.zeros(6)
    reflex_index=np.ones(6)*(-10)
    on_reflex_stance=np.zeros(6)
    reflex_index_stance=np.ones(6)*(-10)
    stance_step_per_reflex=np.ones(6)*(0)
    swing_step_per_reflex=np.ones(6)*(0)
    reflex_count=np.zeros(6)
    reflex_stance_sim=np.zeros(6)
    imu_reflex_real=np.zeros(6)
    sum_leg_all=[]
    reflex_real_all=[]
    on_reflex_all=[]
    traj_error_all=[]
    count_all=[]
    csv_rows=[]
    last_reflex_coef=np.ones(6)
    coef_real=np.zeros(6)
    action=np.zeros(6)
    theta_new_sim=np.zeros(18)
    swing_coef_apply=np.ones(6)  
    imu_coef=np.ones(6)  
    imu_reflex_sim=np.zeros(6)
    
    
    
    swing_coef=2
    stance_coef=20
    stance_coef=1
    step=0
    T=240
    T_count=0
    coef=2
    coef_stance=1
    step=0
    reflex=np.zeros(6)
    # 计数在每个swing or reflex中的步数
    swing_step_count=0
    step_count=0
    
    


    while True:
        count = ser.inWaiting()
        if count >= 108:
            
            if step_count<4:
                start_time_t=time.time()
                
                data1 = ser.read(108)
                print("count",count,time.time())
                data_unpack=struct.unpack('<27f',data1) 
                #print("unpack all",data_unpack,"time:",time.time())
                Position_Read=np.array(data_unpack[0:18])
                IMU_data=np.array(data_unpack[18:21]) #弧度 单位
                relative_z=np.array(data_unpack[21:])
                if step>=1:
                    
                    position_Read_tick=angles_to_tick(Position_Read)
                    flat_cpg_tick=current_pos_tick[step-1] #18
                    traj_error_buf[step-1]=flat_cpg_tick-position_Read_tick
                
                
                theta_sim=goal_pos_sim[step]    
                angles_real=sim_angles_to_real(theta_sim)
                theta_tick=angles_to_tick(angles_real)
                
                last_theta_tick=theta_tick
                
                
                action_theta=theta_tick.tolist()
                action_theta_array=struct.pack('<18f',*action_theta)
                crc=crc16_cal(action_theta_array)
                crc1=crc.to_bytes(2,'big')	
                end=b'\n'
                action_array_crc=action_theta_array+crc1+end  
                ser.write(action_array_crc)  
                
                
                # read feedback
               
                
                on_reflex=np.zeros(6)
                step_count+=1
                if step==239:
                    step=0
                else:
                    step=step+1
                    
                if cpg_index==239:
                    cpg_index=0
                else:
                    cpg_index+=1
                if step%T==int(T/4-1):
                    T_count+=1
                
            else:
                
                start_time_t=time.time()
                
                data1 = ser.read(108)
                print("count",count,time.time())
                data_unpack=struct.unpack('<27f',data1) 
                #print("unpack all",data_unpack,"time:",time.time())
                Position_Read=np.array(data_unpack[0:18]) # 单位 180度
                IMU_data=np.array(data_unpack[18:21]) #弧度 单位
                relative_z=np.array(data_unpack[21:])
                #print("Position_Read",Position_Read,"IMU_data",IMU_data,"relative_z",relative_z)
                
                # judhe whwther or not is a reflex
                # 在判断一开始的reflex 之前就清零
                # judge reflex 
                # swing_step_per_reflex 每个摇摆相开始时置为0  当有reflex时为1  并且在reflex中每隔step +1
                
                
                
                # read feedback
                ## servo plant 也同样运动
                if on_reflex.any() or imu_reflex_sim.any():
                    q_servo_obs_next.put(theta_new_sim)
                    #position_servo_sim=q_pos_read.get(True,10)
                    #position_servo_real=sim_angles_to_real(position_servo_sim)
                    #position_servo_tick=angles_to_tick(position_servo_real)
                
                phase_now=phase[step]
                
                
                ## 实际运行的幅值
                for i in range(6):
                    if phase_now[i]==1:#摇摆相
                        coef_real[i]=(swing_coef_apply[i]*on_reflex[i])/( swing_coef+1)
                    elif phase_now[i]==-1:#支撑相
                        coef_real[i]=(imu_coef[i]*imu_reflex_sim[i])/( stance_coef+1)  # 归一化到1    
                    
                # read feedback
                time_obs_before=time.time()
                #observation,position_Read=read_servos_only(servos,data_read_json,cpg_index,step,IMU_data_angles,coef_real)
                cpg_now=cpg_r[:,:,cpg_index]
                observation=read_servos_only1(Position_Read,relative_z,cpg_index,step,IMU_data,coef_real,phase_now,cpg_now)
                #print("observation time",(time.time()-time_obs_before)*1000)
                q_obs1.put(observation)
                action_flag=0
                
                
                position_Read_tick=angles_to_tick(Position_Read)
                flat_cpg_tick=current_pos_tick[step] #18
                
                on_reflex_real=sim_reflex_leg_to_real(on_reflex)
                
                
                if q_act.empty()==False:
                    action=q_act.get(True,1)
                    print("action!!")
                    action_flag=1
                print("action time",(time.time()-start_time_t)*1000)
                if on_reflex.any() or imu_reflex_sim.any():
                    #q_servo_obs_next.put(theta_new_sim)
                    position_servo_sim=q_pos_read.get(True,10)
                    position_servo_real=sim_angles_to_real(position_servo_sim)
                    position_servo_tick=angles_to_tick(position_servo_real)
                if q_act.empty()==False:
                    action=q_act.get(True,1)
                    print("action!!")
                    action_flag=1
                
                
                time4=time.time()
                print("read pos  time",(time4-start_time_t)*1000)
                
                
                imu_reflex_real=sim_reflex_leg_to_real(imu_reflex_sim)
                
                if on_reflex_real.any() or imu_reflex_real.any():
                    for i in range(6):
                        if on_reflex_real[i]==1:
                            flat_cpg_tick[3*i:3*i+3]=position_servo_tick[3*i:3*i+3]
                        elif imu_reflex_real[i]==1:
                            flat_cpg_tick[3*i:3*i+3]=position_servo_tick[3*i:3*i+3]
            
                ## buffer 升级
                for i_index in range(3):
                    traj_error_buf[i_index]=traj_error_buf[i_index+1]
                traj_error_buf[3]=flat_cpg_tick-position_Read_tick
                
                
                
                
                
                
                if step==239:
                    step=0
                else:
                    step=step+1
                    
                if cpg_index==239:
                    cpg_index=0
                else:
                    cpg_index+=1
                if step%T==int(T/4-1):
                    T_count+=1
                step_count+=1
                
                if step_count>=T*num_T-1:
                    print("end all~~~~~~~~~~~~~~~~`")
                    break

                
                
                
                
                ## 新的step  判断REFLEX
                
                
                
                
                phase_now=phase[step]
                last_phase=phase[step-1]
                
                reflex,sum_leg=judge_reflex(traj_error_buf)
                reflex_sim=real_reflex_leg_to_sim(reflex)
                reflex_sim=swing_reflex(reflex_sim,phase_now)
                
                reflex_sim_new=judge_new_reflex(reflex_sim,swing_step_per_reflex)
                
                
                print("sum_leg",sum_leg,)
                
                #print("count:",count,"reflex",reflex,"on reflex",on_reflex)
                # on_reflex 在sim的基础上
                
                #on_reflex=swing_reflex(on_reflex,phase_now)
                #on_reflex_stance=stance_reflex(on_reflex_stance,phase_now)
                if reflex_sim.any():
                    reflex_index[reflex_sim>0]=T_count
                    on_reflex[reflex_sim>0]=1
                
                if reflex_sim_new.any():
                    swing_step_per_reflex[reflex_sim_new>0]=0
                    
                if on_reflex.any():
                    swing_step_per_reflex[on_reflex>0]+=1
                
                    
                swing_step_per_reflex[swing_step_per_reflex>int(T/2-1)]=0
                
                
                
                # swing 和stance  交界处 error 清零
                swing_step_per_reflex[last_phase*phase_now<0]=0
                
                swing_step_count+=1# 在摇摆相的第几步
                if (last_phase*phase_now)[0]<1:
                    traj_error_buf=np.zeros_like(traj_error_buf)
                    swing_step_count=0
                    on_reflex=np.zeros_like(on_reflex)
                    reflex_count=np.zeros_like(on_reflex)
                    
                for i in range(6):# 当进入下一周期后 默认不在reflex范围内
                    if (T_count-reflex_index[i])==1 and on_reflex[i]>0:
                        on_reflex[i]=0
                        reflex_index[i]=-2
                        swing_step_per_reflex[i]=0
                
                # imu reflex   
                #imu_reflex=judge_imu_reflex2(IMU_data_angles,phase_now)
                
                
                
                time2=time.time()
                print("reflex time",(time2-start_time_t)*1000)
                
                if (swing_step_per_reflex==1).any() :
                    reflex_count[swing_step_per_reflex==1]+=1
                    position_read_sim=real_angles_to_sim(Position_Read)
                    q_servo_obs_now.put(position_read_sim.flatten())
                    #position_servo_sim=q_pos_read.get(True,10)
                    #position_servo_real=sim_angles_to_real(position_servo_sim)
                    #position_servo_tick=angles_to_tick(position_servo_real)
                    # TODO
                    #for index_i in range(30):
                    #    set_position_multi(position_Read,robot_servo,servo_client)
                    #position_servo=get_position(robot_servo,servo_client)
                        
                # 将reflex  都转换到仿真的维度中  使得能够与phase 相关
                #on_reflex_sim=real_reflex_leg_to_sim(on_reflex)
                #on_reflex_stance_sim=real_reflex_leg_to_sim(on_reflex_stance)     
                
                #theta_new_sim=get_reflex_theta(theta_sim,coef,reflex_sim,on_reflex_sim,phase_now,)
                
                
                
                IMU_data_angles=np.array(IMU_data)/math.pi*180
                imu_reflex_sim=judge_imu_reflex2(IMU_data_angles,phase_now)
                if action_flag==0:
                    action=q_act.get(True,8)
                    print("action!!")
                
                action_coef=np.zeros(6)
                for coef_index in range(6):
                    if phase_now[coef_index]==1:# 摇摆相
                        action_coef[coef_index]=(action[coef_index]* swing_coef)
                    elif phase_now[coef_index]==-1:# 支撑相
                        action_coef[coef_index]=(action[coef_index]* stance_coef)
                if swing_step_count<10:
                    coef=1*np.ones(6)
                        #coef_stance=1
                else:
                    #coef=np.clip(1*np.ones(6)+coef_delta*(reflex_count),0,3)
                        #coef_stance=1.5
                    coef=np.clip(1*np.ones(6)+action_coef,1,3)
                    #print("reflex sim",reflex_sim,"on reflex",on_reflex)
                
                for i in range(6):
                    if swing_step_per_reflex[i]==1:
                        if reflex_sim_new[i]==1:# 如果是二次反射
                            swing_coef_apply[i]=np.clip(coef[i]+1,1,3)
                        else:
                            swing_coef_apply[i]=coef[i]
                        last_reflex_coef[i]=swing_coef_apply[i]
                    else:
                        swing_coef_apply[i]=last_reflex_coef[i]
                    
                    
            
                #print("traj_error_buf",traj_error_buf,)
                imu_coef=action_coef+np.ones(6)
                coef_stance=np.zeros(6)
                reflex_stance_sim=np.zeros(6)
                
                
                    
                    
                
                theta_sim=goal_pos_sim[step] 
                
                theta_new_sim0=get_reflex_theta_all2(theta_sim,swing_coef_apply,coef_stance,reflex_sim,on_reflex,phase_now,swing_step_per_reflex)
                theta_new_sim=imu_roll_reflex2(imu_reflex_sim,phase_now,theta_new_sim0,imu_coef) 
                
                
                
                angles_real=sim_angles_to_real(theta_new_sim)
                theta_tick=angles_to_tick(angles_real)
                theta_new_tick=imu_reflex_smooth(theta_tick,last_theta_tick,phase_now)
                last_theta_tick=theta_new_tick
                
                
                action_theta=theta_new_tick.tolist()
                action_theta_array=struct.pack('<18f',*action_theta)
                crc=crc16_cal(action_theta_array)
                crc1=crc.to_bytes(2,'big')	
                
                end=b'\n'
                action_array_crc=action_theta_array+crc1+end  
                ser.write(action_array_crc)  
        
                time3=time.time()
                print("write time",(time3-start_time_t)*1000)
                
                
                
                
            
                # csv writer
                #traj_error_all.append(flat_cpg_tick-position_Read_tick)
                #count_all.append(count)
                #reflex_real_all.append(reflex)
                #on_reflex_all.append(on_reflex)
                #sum_leg_all.append(sum_leg)
                on_reflex2=copy.copy(on_reflex)
                
            
                reflex_index2=copy.copy(reflex_index)
        
                
                error_now=copy.copy(traj_error_buf[3])
                
                last_time1=(time.time()-start_time_t)*1000
                swing_coef_apply2=copy.copy(copy.copy(swing_coef_apply))
                
                csv_row=[]
                csv_row=[step_count,T_count,sum_leg,reflex_sim,on_reflex2,reflex_index2,swing_step_count,flat_cpg_tick,position_Read_tick,IMU_data_angles,error_now,swing_step_count,coef,imu_reflex_sim,last_time1,action,coef,imu_coef,swing_coef_apply2,theta_tick]
                csv_rows.append(csv_row)
                
                print("end time",(time.time()-start_time_t)*1000)
                
                
                
                
                
                
                
                
                
                
                
            
                #print("current",current_read,"\n")
                print("error now",error_now)
                #print("reflex_stance",reflex_stance_sim,"on reflex",on_reflex_stance,'reflex_index_stance',reflex_index_stance,'T_count',)
                #while (time.time()-start_time_t)*1000<20.00:
                #    1
                
                if (time.time()-start_time_t)*1000>=20.0:
                    print("---------------------over time --------------")
                print("last time",(time.time()-start_time_t)*1000,"count:",step_count,"reflex",reflex_sim,"on reflex",on_reflex,'reflex_index',reflex_index,'T_count',T_count,"imu_reflex",imu_reflex_real)
                #print("end time",(time5-start_time_t)*1000)
                
        
        
        
        
            
        
        
        
    
    # csv
    
    with open('data_reflex_990_5.csv', mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['count','T_count','sum_leg','reflex_sim','reflex_stance_sim','on_reflex2','on_reflex_stance2','reflex_index2','reflex_index_stance2','swing_step_count','flat_cpg_tick','position_Read_tick','IMU_data'])
        writer.writerows(csv_rows)

    data = {'positions': positions}
    data_json = json.dumps(data, cls=NumpyArrayEncoder)
    



import numpy as np

from Servos import *

if __name__ == '__main__':
    
    num_T=8
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
    q_servo_obs_now= Queue()
    q_servo_obs_next=Queue()
    q_pos_read= Queue()
    q_obs1=Queue()
    
    
    ser = serial.Serial("/dev/ttyAMA0",115200*16,parity='E',timeout=0.00001)
    if not ser.isOpen():
        print("open failed")
    else:
        print("open success: ")
        print(ser)
    
    
    model1_dir="/home/fast09/Desktop0/DynamixelSDK-3.7.31/python/tests/protocol2_0/distill_model/action_net_one_BC_mlp_new2_1_990.pt"
    # socket

    
   
    time.sleep(0.1)
    servo_client = Process(target=servo_pybullet,args=(q_servo_obs_now,q_servo_obs_next,q_pos_read,) )
    servo_client.start()
    actor1 = Process(target=load_and_actor,args=(model1_dir,q_obs1,q_act) )
    actor1.start()
    time.sleep(2)
    reflex_(ser,q_servo_obs_now,q_servo_obs_next,q_pos_read,q_obs1,q_act,num_T)
    #Reflex_ = Process(target=reflex_,args=(q_imu,) )
    #Reflex_.start()
    
    
    
    
        
