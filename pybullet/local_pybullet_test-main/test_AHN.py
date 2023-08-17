import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import math
from CPGs import *
import random
import json


# 插值
def interp(theta_0, theta_1, n):
    theta_n = np.zeros((n, 6, 3))
    # theta_n=theta_0+(theta_1-theta_0)/n
    for j in range(n):
        theta_n[j, :, :] = theta_0 + (theta_1 - theta_0) / n * (j + 1)

    return theta_n


def setbc_position(theta_b):
    for j in range(6):
        if j != 3:
            p.setJointMotorControl2(robot, 3 * j, p.POSITION_CONTROL, theta_b[j])
        if j == 3:
            p.setJointMotorControl2(robot, 3 * j, p.POSITION_CONTROL, -theta_b[j])


def set_nbc_position(theta_k, n):
    for j in range(n):
        setbc_position(theta_k[j])
        p.stepSimulation()


def set_n_position(theta_k, n):
    for j in range(n):
        set_position(theta_k[j, :, :])
        p.stepSimulation()


def set_position(theta):
    for j in range(6):
        p.setJointMotorControl2(robot, 3 * j, p.POSITION_CONTROL, theta[j, 0])
        p.setJointMotorControl2(robot, 3 * j + 1, p.POSITION_CONTROL, theta[j, 1])
        p.setJointMotorControl2(robot, 3 * j + 2, p.POSITION_CONTROL, theta[j, 2])


def get_contactPoints(p):
    result = np.zeros((6))
    contact_norm=np.zeros((6,3))
    phase = cpgs.get_phase(cpgs.old_zz)
    for j in range(6):
        foot_contacts = []
        contact_force_v = 0
        contact_force_l = 0

        for body_1 in box:
            foot_contact = p.getContactPoints(bodyA=robot, bodyB=body_1, linkIndexA=3 * j + 2)
            if len(foot_contact):
                foot_contacts.append(foot_contact)
        if not len(foot_contacts):
            link_name = (p.getJointInfo(robot, 3 * j + 2)[12]).decode()
            contact_force_v = 0
            contact_force_l = 0
        else:
            for f_contacts in foot_contacts:
                for f_contact in f_contacts:
                    link_index = f_contact[3]
                    contact_normal = f_contact[7]# 接触方向
                    contact_force_v = f_contact[9] * contact_normal[2] / np.linalg.norm(contact_normal)
                    contact_force_l = f_contact[9] * np.linalg.norm(contact_normal[0:1]) / np.linalg.norm(
                        contact_normal)
                    # print(contact_normal)
                    if link_index >= 0:
                        link_name = (p.getJointInfo(robot, link_index)[12]).decode()
                    else:
                        link_name = 'base'

        # result.append((link_name, f_contact[9]))
        result[j] =  1 *contact_force_v
    return result


def __get_observation():
    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    (x, y, z, w) = baseOri
    # print(basePos)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    IMU = np.array([roll, pitch, yaw])
    robot_joints_states = p.getJointStates(robot, jointIndices=range(18))
    robot_joint_positions = np.array([x[0] for x in robot_joints_states])
    # 水平和侧向的接触力
    contact_forces = np.zeros(12)
    phase = cpgs.get_phase(cpgs.old_zz)
    for j in range(6):
        foot_contacts = []
        contact_force_v = 0
        contact_force_l = 0
        for body_1 in box:
            foot_contact = p.getContactPoints(bodyA=robot, bodyB=body_1, linkIndexA=3 * j + 2)
            if len(foot_contact):
                foot_contacts.append(foot_contact)
        if not len(foot_contacts):
            contact_force_v = 0
            contact_force_l = 0
        else:
            for f_contacts in foot_contacts:
                for f_contact in f_contacts:
                    contact_normal = f_contact[7]
                    contact_force_v += f_contact[9] * contact_normal[2] / np.linalg.norm(contact_normal)
                    contact_force_l += f_contact[9] * np.linalg.norm(contact_normal[0:1]) / np.linalg.norm(
                        contact_normal)

        contact_forces[2 * j] = phase[j] * contact_force_v
        contact_forces[2 * j + 1] = contact_force_l
    # print(contact_forces)
    # print(phase)
    observation = np.append(robot_joint_positions, contact_forces)
    observation = np.append(observation, IMU)

    return observation

def __get_observation1(data_read):
    agent_num=6
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
# IMU的数据
    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    (x, y, z, w) = baseOri
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    IMU = np.array([roll, pitch, yaw])

    observation_temp=[]
    phase = cpgs.get_phase(cpgs.old_zz)
    theta_old = cpgs.cpg2theta_reshape(cpgs.old_zz)

    for agent_index in range(agent_num):
        JointIndicies_agenti=[3*agent_index,3*agent_index+1,3*agent_index+2]
        robot_joints_states_agenti = p.getJointStates(robot, jointIndices=JointIndicies_agenti)
        robot_joint_positions_agenti = np.array([x[0] for x in robot_joints_states_agenti])
        robot_joint_Torque_agenti = np.array([x[3] for x in robot_joints_states_agenti])
        world_end_position=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True)[0]
        world_end_ori=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True)[0]
        link_name = (p.getJointInfo(robot, 3*agent_index+2)[12]).decode()
        #print("world end position",world_end_position)
        if agent_index>=3:
            end_pos,end_ori=get_forward_pos(-robot_joint_positions_agenti)
        else:
            end_pos,end_ori=get_forward_pos(robot_joint_positions_agenti)
        print("world end position",end_pos)
        #print(link_name)
        # 误差的error
        theta_error=robot_joint_positions_agenti-theta_old[agent_index,:]



        # 相位信息
        phase_continus1=cpgs.old_zz[agent_index,2]
        phase_continus2=cpgs.old_zz[agent_index,3]
        phase_continus=cpgs.old_zz[agent_index,2:4]
        force_sin=force_r[abs(cpg_r[agent_index,2,:]-phase_continus1)<1e-3,agent_index]
        force_cos=force_r[abs(cpg_r[agent_index,3,:]-phase_continus2)<1e-3,agent_index]
        #force_t=np.intersect1d(force_cos,force_sin)[0]
        



        

        # 竖直方向的接触力
        foot_contacts = []
        contact_force_v = 0
        contact_force_l = 0
        for body_1 in box:
            foot_contact = p.getContactPoints(bodyA=robot, bodyB=body_1, linkIndexA=3 * agent_index + 2)
            if len(foot_contact):
                foot_contacts.append(foot_contact)
        if not len(foot_contacts):
            contact_force_v = 0
            contact_force_l = 0
        else:
            for f_contacts in foot_contacts:
                for f_contact in f_contacts:
                    contact_normal = f_contact[7]
                    contact_force_v += f_contact[9] * contact_normal[2] / np.linalg.norm(contact_normal)
                    contact_force_l += f_contact[9] * np.linalg.norm(contact_normal[0:1]) / np.linalg.norm(contact_normal)
        #contact_force_diff=contact_force_v-force_t
        #print("contact_force_diff ",contact_force_diff)
        contact_foot=phase_continus1*contact_force_v
        if contact_foot>0:
            print("!! collision")


        observation_agenti = np.append(robot_joint_positions_agenti, robot_joint_Torque_agenti)
        observation_agenti = np.append(observation_agenti,contact_force_v)
        #observation_agenti = np.append(observation_agenti,contact_force_diff)
        observation_agenti = np.append(observation_agenti,world_end_position)
        observation_agenti = np.append(observation_agenti,world_end_ori)
        observation_agenti = np.append(observation_agenti,phase_continus)
        observation_agenti = np.append(observation_agenti,theta_error)


        observation_temp.append(observation_agenti)

    observation = np.array(np.vstack(observation_temp),dtype=np.float64)
    return observation        


def get_forward_pos(pos):
    theta_1=0+pos[0]
    theta_2=-(90-21.36-0)/180*3.1415926+pos[1]
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


def __get_joints_torque():
        joint_torque = np.zeros(18)
        for j in range(18):
            joint_torque[j] = p.getJointState(robot, j)[3]
        return joint_torque

def  cal_dhs(data_read,count):
    agent_num=6
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
# IMU的数据
    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    (x, y, z, w) = baseOri
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    IMU = np.array([roll, pitch, yaw])

    observation_temp=[]
    phase = cpgs.get_phase(cpgs.old_zz)
    theta_old = cpgs.cpg2theta_reshape(cpgs.old_zz)
    offset_final_all=[]

    for agent_index in range(agent_num):
        JointIndicies_agenti=[3*agent_index,3*agent_index+1,3*agent_index+2]
        robot_joints_states_agenti = p.getJointStates(robot, jointIndices=JointIndicies_agenti)
        robot_joint_positions_agenti = np.array([x[0] for x in robot_joints_states_agenti])
        robot_joint_Torque_agenti = np.array([x[3] for x in robot_joints_states_agenti])
        world_end_position=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True)[0]
        world_end_ori=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True)[0]
        link_name = (p.getJointInfo(robot, 3*agent_index+2)[12]).decode()
        #print("world end position",world_end_position)
        if agent_index>=3:
            end_pos,end_ori=get_forward_pos(-robot_joint_positions_agenti)
        else:
            end_pos,end_ori=get_forward_pos(robot_joint_positions_agenti)
        #print("world end position",end_pos)
        #print(link_name)
        # 误差的error
        theta_error=robot_joint_positions_agenti-theta_old[agent_index,:]



        # 相位信息
        phase_continus1=cpgs.old_zz[agent_index,2]
        phase_continus2=cpgs.old_zz[agent_index,3]
        phase_continus=cpgs.old_zz[agent_index,2:4]
        force_sin=force_r[abs(cpg_r[agent_index,2,:]-phase_continus1)<4e-3,agent_index]
        force_cos=force_r[abs(cpg_r[agent_index,3,:]-phase_continus2)<4e-3,agent_index]
        force_t=np.intersect1d(force_cos,force_sin)[0]
        



        

        # 竖直方向的接触力
        foot_contacts = []
        contact_force_v = 0
        contact_force_l = 0
        for body_1 in box:
            foot_contact = p.getContactPoints(bodyA=robot, bodyB=body_1, linkIndexA=3 * agent_index + 2)
            if len(foot_contact):
                foot_contacts.append(foot_contact)
        if not len(foot_contacts):
            contact_force_v = 0
            contact_force_l = 0
        else:
            for f_contacts in foot_contacts:
                for f_contact in f_contacts:
                    contact_normal = f_contact[7]
                    contact_force_v += f_contact[9] * contact_normal[2] / np.linalg.norm(contact_normal)
                    contact_force_l += f_contact[9] * np.linalg.norm(contact_normal[0:1]) / np.linalg.norm(contact_normal)
        contact_force_diff=-contact_force_v+force_t
        #print("contact_force_diff ",contact_force_diff)

        if count<buffer_DHS:
            s_buffer_dhs[count,agent_index]= contact_force_diff
            r_l_sum=0
            for index_i in range(count+1):
                r_l_sum+=s_buffer_dhs[index_i,agent_index]
            DHS_S=r_l_sum/(count+1)

        else:
            for index_i in range(buffer_DHS-1):
                s_buffer_dhs[index_i,agent_index]=s_buffer_dhs[index_i+1,agent_index]
            s_buffer_dhs[buffer_DHS-1,agent_index]=contact_force_diff
            DHS_S=sum(s_buffer_dhs[:,agent_index])/buffer_DHS
        alpha=0.8
        beta=0.9795
        H_g=alpha*DHS_S/pow(1+pow(DHS_S,2),0.5)
        DHS_HC_buffer[1,agent_index]=DHS_HC_buffer[0,agent_index]
        Hc_dhs=(beta*DHS_HC_buffer[0,agent_index]+H_g)/(1+0.0005)
        DHS_HC_buffer[0,agent_index]=Hc_dhs
        max_hc=max(DHS_HC_buffer[2,agent_index],Hc_dhs)
        min_hc=min(DHS_HC_buffer[3,agent_index],Hc_dhs)
        DHS_HC_buffer[2,agent_index]=max_hc
        DHS_HC_buffer[3,agent_index]=min_hc
        max_hc=max(max_hc,1)

        receptor_dhs=(0.188-0)*(Hc_dhs-0)/(max_hc-min_hc)+0
        offset=receptor_dhs
        


        ### CHS
        basePos, baseOri = p.getBasePositionAndOrientation(robot)
        linVel,angVel=p.getBaseVelocity(robot)
        (new_x, new_y, new_z) = basePos
        (x, y, z, w) = baseOri
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        IMU = np.array([roll, pitch, yaw])
        CHS_x=0
        if agent_index<3:
            if roll <0:
                CHS_x=-roll
        else:
            if roll>0:
                CHS_x=roll
        if count<buffer_CHS:
            s_buffer_chs[count,agent_index]= CHS_x
            r_l_sum=0
            for index_i in range(count+1):
                r_l_sum+=s_buffer_chs[index_i,agent_index]
            CHS_s=r_l_sum/(count+1)

        else:
            for index_i in range(buffer_CHS-1):
                s_buffer_chs[index_i,agent_index]=s_buffer_chs[index_i+1,agent_index]
            s_buffer_chs[buffer_DHS-1,agent_index]=CHS_x
            CHS_s=sum(s_buffer_chs[:,agent_index])/buffer_CHS
        
        H_g_chs=alpha*CHS_s/pow(1+pow(CHS_s,2),0.5)
        CHS_HC_buffer[1,agent_index]=CHS_HC_buffer[0,agent_index]
        Hc_chs=(beta*CHS_HC_buffer[0,agent_index]+H_g_chs)/(1+0.0005)
        CHS_HC_buffer[0,agent_index]=Hc_chs

        max_hc=max(CHS_HC_buffer[2,agent_index],Hc_dhs)
        min_hc=min(CHS_HC_buffer[3,agent_index],Hc_dhs)
        CHS_HC_buffer[2,agent_index]=max_hc
        CHS_HC_buffer[3,agent_index]=min_hc
        max_hc=max(max_hc,1)

        receptor_dhs=(Hc_chs-0)/(max_hc-min_hc)+0
        K_t=receptor_dhs+1
        print("contact_force_diff ",contact_force_diff,"roll",roll)
        offset_final=K_t*offset
        offset_final_all.append(offset_final)
    print(offset_final_all)
    return offset_final_all

def AHN_theta_adaptaion(theta1,final_offset_a):
    
    for j in range(6):
        if j<3:
            flag=-1
        else:
            flag=1
        theta1[j, 1]=theta1[j,1]+flag*final_offset_a[j]
        theta1[j, 2]=theta1[j,2]+flag*final_offset_a[j]

    return theta1
        
            



def __get_reward(old_base_po,count):
    (old_x,old_y,old_z)=old_base_po
    state = __get_observation()
    contact_force = state[18:30]
    contact_force[contact_force < 0] = 0
    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    linVel,angVel=p.getBaseVelocity(robot)
    (new_x, new_y, new_z) = basePos
    (x, y, z, w) = baseOri
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    IMU = np.array([roll, pitch, yaw])
    joint_torques = __get_joints_torque()
    # 奖励
    if count<buffer_num:
        reward_buff[count]= pow((new_y - old_y) / dt, 2) 
        r_l_sum=0
        for index_i in range(count):
            r_l_sum+=reward_buff[index_i]
        r_linear_forward=r_l_sum/(count+1)

    else:
        for index_i in range(buffer_num-1):
            reward_buff[index_i]=reward_buff[index_i+1]
        reward_buff[buffer_num-1]=pow((new_y - old_y) / dt, 2) 
        r_linear_forward=sum(reward_buff)/buffer_num


    
    
    # 　惩罚侧向的偏移

    if count<buffer_num_x:
        reward_buff_x[count]= -abs((new_x - old_x) / dt)- abs(new_x) * dt * 20 
        r_l_sum=0
        for index_i in range(count):
            r_l_sum+=reward_buff_x[index_i]
        r_lateral_move=r_l_sum/(count+1)

    else:
        for index_i in range(buffer_num_x-1):
            reward_buff_x[index_i]=reward_buff_x[index_i+1]
        reward_buff_x[buffer_num_x-1]=-abs((new_x - old_x) / dt)- abs(new_x) * dt * 20
        r_lateral_move=sum(reward_buff_x)/buffer_num_x
    #r_lateral_move = -abs((new_x - old_x) / dt)- abs(new_x) * dt * 20
    # 惩罚不稳定性
    r_instablity = -0.3 * abs(sum(IMU)) - 3 * abs(IMU[-1])
    
    # 惩罚机器人与障碍物的碰撞，以及没有及时抬脚,惩罚摇摆相位出现力反馈
    if count<buffer_num_c:
        reward_buff_c[count]= -np.linalg.norm(contact_force)
        r_l_sum=0
        for index_i in range(count):
            r_l_sum+=reward_buff_c[index_i]
        r_collision=r_l_sum/(count+1)

    else:
        for index_i in range(buffer_num_c-1):
            reward_buff_c[index_i]=reward_buff_c[index_i+1]
        reward_buff_c[buffer_num_c-1]=-np.linalg.norm(contact_force)
        r_collision=sum(reward_buff_c)/buffer_num_c
    #r_collision = -np.linalg.norm(contact_force)
    #扭矩
    if count<buffer_num_t:
        reward_buff_t[count]= -np.linalg.norm(joint_torques)
        r_l_sum=0
        for index_i in range(count):
            r_l_sum+=reward_buff_t[index_i]
        r_torque=r_l_sum/(count+1)

    else:
        for index_i in range(buffer_num_t-1):
            reward_buff_t[index_i]=reward_buff_t[index_i+1]
        reward_buff_t[buffer_num_t-1]=-np.linalg.norm(joint_torques)
        r_torque=sum(reward_buff_t)/buffer_num_t





    #r_torque = -np.linalg.norm(joint_torques)
    reward=np.array([r_linear_forward,r_lateral_move,r_instablity,r_collision])
    
    w_linear_forward = 2000
    w_lateral_move = 80
    w_instablity = 8
    w_collision = 0.5
    w_torque = 4  
    reward2 = np.array([w_linear_forward * r_linear_forward, w_lateral_move * r_lateral_move, w_instablity * r_instablity,
                        max(w_collision * r_collision, -20), max(w_torque * r_torque, -20)])
    
    reward_all=sum(reward2)
    if new_y>0.8:
        #print("y",new_y)
        reward_all+=10
    else:
        if new_y<0.5:
            #print("y",new_y)
            reward_all-=20

    #print(reward_all)
    return reward2

p.connect(p.GUI)  # 连接到仿真服务器
p.setGravity(0, 0, -9.8)  # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath())  # 设置pybullet_data的文件路径

# 加载地面
floor = p.loadURDF("plane.urdf")

# mini cheetah的初始位置
startPos = [0, 0, 0.5]

# 加载urdf文件
robot = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos)

box = []
# rewards buffer
max_hc=0
min_hc=0
buffer_num=240
buffer_num_x=90
buffer_num_c=90
buffer_num_t=90
reward_buff=np.zeros(buffer_num)
reward_buff_x=np.zeros(buffer_num_x)
reward_buff_c=np.zeros(buffer_num_c)
reward_buff_t=np.zeros(buffer_num_t)
buffer_DHS=150
s_buffer_dhs=np.zeros((buffer_DHS,6))
DHS_HC_buffer=np.zeros((4,6))# t//t-1//max//min
buffer_CHS=150
s_buffer_chs=np.zeros((buffer_CHS,6))
CHS_HC_buffer=np.zeros((4,6))# t//t-1

for j in range(10):
    x_ran = random.uniform(-0.6, 0.6)
    y_ran = random.uniform(0, 0.5)
    Pos = [x_ran, y_ran, 0.2]
    #box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos))
Pos = [0.25, 0.4, 0.2]
# box1=p.loadURDF("box_80_60_30/box2.urdf",Pos)
#box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos))
Pos2 = [0.4, 0.7, 0.2]
# box2=p.loadURDF("box_80_60_30/box2.urdf",Pos2)
#box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos2))
Pos3 = [-3.5, 0.8, 0.2]
# box3=p.loadURDF("box_80_60_30/box2.urdf",Pos3)
box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos3))
#box3=p.loadURDF("box_40_30_20/box1.urdf",Pos3)

box.append(floor)

for j in range(18):
    p.changeDynamics(robot,j,lateralFriction=1)
    dyn = p.getDynamicsInfo(robot,j)
    f_friction_lateral=dyn[1]
    print(f_friction_lateral,j)
print("box")
for j in range(1):
    p.changeDynamics(box[j], -1, lateralFriction=1)
    dyn = p.getDynamicsInfo(box[j],-1)
    f_friction_lateral=dyn[1]
    print(f_friction_lateral,j)
# 获取关节数量
numJoints = p.getNumJoints(robot)
# 设置机器人的视觉效果，这里参数-1指的是机器人的base link。
# 我们可以通过rgba值改变机器人相关link的颜色
p.changeVisualShape(robot, -1, rgbaColor=[1, 1, 1, 1])

cf_base = 0.35  # 右边的零位状态　左边增加负号
ft_base = -0.35

pos = [0, -cf_base, -ft_base, 0, -cf_base, -ft_base, 0, -cf_base, -ft_base,
       0, cf_base, ft_base, 0, cf_base, ft_base, 0, cf_base, ft_base]

p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                             cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
# 设置每个link的颜色，并且设置初始关节角度
for j in range(numJoints):
    force = 30
    p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, pos[j])

# 　调整至零位
dt = 1. / 240.
p.setTimeStep(dt)

p.stepSimulation()

state_t = 0
amp = 0.35
amp2 = 0.15
fre = 0.5
df = 0.3
PH = [0, 0.5, 0, 0.5, 0, 0.5]
dt = 1.0 / 240.0
theta_phase = [[0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0], [0, 0.5, 0, 0.5, 0, 0.5],
               [-0.5, 0, -0.5, 0, -0.5, 0],
               [0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0]]


class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


# 计算初始位置并且设置
z0 = np.ones(12) * 0.5
cpgs = CPGs_O(theta_phase, dt, 0.5)

with open('initial.json', 'r') as f:
    data_read = json.load(f)
    theta0 = np.asarray(data_read['joint_angles'])
    zz0 = np.asarray(data_read['CPG_output'])

# zz0 = cpgs.cpg_begin_all(z0)
# theta0 = cpgs.cpg2theta_reshape(zz0)

with open('force_real6.json', 'r') as f:
    data_read = json.load(f)
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    theta_ini_r=np.asarray(data_read['theta_ini'])
    ini_index_r=np.asarray(data_read['ini_index'])

set_position(theta_ini_r)
p.stepSimulation()
for j in range(240 * 1):
    p.stepSimulation()

tf = 10
start_T = time()
zz_n = cpgs.two_layer_out_all(zz0, 0, tf)
t_eval = np.arange(0.0, tf, dt)
end_T = time()
print(end_T - start_T)
cpgs.old_zz=cpg_r[:,:,ini_index_r]

theta_f = np.zeros((6, 3))
l1_bc = []
l1_cf = []
l1_ft = []
r3_bc = []
r3_cf = []
r3_ft = []
T = []

count = 0
contacts = np.zeros((1, 6))
T = [0]
theta1_n = np.zeros((1, 6, 3))
torque_n = np.zeros((1, 6, 3))
phase1_n = np.zeros((1, 6))
reward_n=np.zeros((1, 5))
reward_n_all=[0]
count = 0
cpg_index=ini_index_r
contacts = np.zeros((1, 6))
T = [0]
theta1_n = np.zeros((1, 6, 3))
phase1_n = np.zeros((1, 6))
reward_n=np.zeros((1, 5))
reward_n_all=[0]

for t in t_eval:
    start_T = time()
    #__get_observation1(data_read,cpg_index)
    end_T = time()
    #print("obs:",end_T-start_T)
    if cpg_index<239:
        cpg_index+=1
    else:
        cpg_index=0



    
    theta1=theta_r[cpg_index,:,:]
    cpgs.old_zz = cpg_r[:,:,cpg_index]
    start_T = time()
    phase1 = cpgs.get_phase(cpgs.old_zz )
    phase1_n = np.append(phase1_n, np.resize(phase1, (1, 6)), axis=0)
    theta1_n = np.append(theta1_n, np.resize(theta1, (1, 6, 3)), axis=0)
    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    theta1=np.squeeze(theta1)
    

    final_offset_a=cal_dhs(data_read,count)
    theta1=AHN_theta_adaptaion(theta1,final_offset_a)
    set_position(theta1)
    # print(phase1)
    # print(t)
    p.stepSimulation()
    count += 1
    location, _ = p.getBasePositionAndOrientation(robot)
    #p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0,
                                 #cameraPitch=-60, cameraTargetPosition=location)
    obser = __get_observation()
    # print(obser)
    contact_result = get_contactPoints(p)
    

    contacts = np.append(contacts, np.resize(contact_result, (1, 6)), axis=0)
    JointIndicies_agenti=range(18)
    robot_joints_states_agenti = p.getJointStates(robot, jointIndices=JointIndicies_agenti)
    robot_joint_positions_agenti = np.array([x[0] for x in robot_joints_states_agenti])
    robot_joint_Torque_agenti = np.array([x[3] for x in robot_joints_states_agenti])
    torque_n = np.append(torque_n, np.resize(robot_joint_Torque_agenti, (1, 6, 3)), axis=0)
    T.append(t)
    reward1=__get_reward(basePos,count)
    reward_all = sum(reward1)
    reward_n = np.append(reward_n, np.resize(reward1, (1, 5)), axis=0)
    reward_n_all.append(reward_all)

basePos, baseOri = p.getBasePositionAndOrientation(robot)
    
(new_x, new_y, new_z) = basePos

print("x y z",basePos)
'''
plt.Figure()
for j in range(6):
    plt.subplot(3, 2, j+1)
    plt.plot(T[240:240*6], zz_n[j,0,240:240*6])
    




plt.Figure()
for j in range(6):
    plt.subplot(3, 2, j+1)
    plt.plot(T[240:240*4], contacts[240:240*4, j],'-r',T[240:240*4],phase1_n[240:240*4,j],'-b',T[240:240*4], zz_n[j,2,240:240*4],'y')
    


plt.show()
'''
class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

theta_ini=theta_r[abs(theta_r[:,0,0])<1e-3][0]
ini_index=np.argwhere(abs(theta_r[:,0,0])<1e-3)[0]
contact1=(contacts[162:402,:]+contacts[162+240*1:402+240*1,:]+contacts[162+240*2:402+240*2,:]+contacts[162+240*3:402+240*3,:])/4
data = {'contact_force': contact1,'phase':phase1_n[162:402,:],'cpg':zz_n[:,:,162:402],'theta':theta1_n[162:402,:,:],'theta_ini':theta_ini,'ini_index':ini_index,'torque_n':torque_n[162:402,:]}
data_json = json.dumps(data, cls=NumpyArrayEncoder)
#print(f'contact force:{data_json}')

# 写入json文件

with open('force_real7.json', 'w') as f:
    json.dump(data, f, cls=NumpyArrayEncoder)


with open('force_real7.json', 'r') as f:
    data_read = json.load(f)
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    theta_ini_r=np.asarray(data_read['theta_ini'])
    ini_index_r=np.asarray(data_read['ini_index'])
    torque_n=np.asarray(data_read['torque_n'])

t_e=np.arange(0.0, 1, dt)


plt.Figure()
for j in range(6):
    plt.subplot(3, 2, j + 1)
    plt.plot(t_e, force_r[:, j],'r',t_e, phase_r[:, j],'b',t_e, cpg_r[j,2,:],'g',t_e, theta_r[:, j,2])

plt.show()

plt.Figure()
for j in range(6):
    plt.subplot(3, 2, j + 1)
    plt.plot(t_e, theta_r[:, j,1])

plt.show()

plt.Figure()
for j in range(5):
    plt.subplot(3, 2, j + 1)
    plt.plot(T, reward_n[:, j])


plt.subplot(3, 2, 6)
plt.plot(T, reward_n_all)
plt.show()