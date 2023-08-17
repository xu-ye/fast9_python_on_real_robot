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
    result = np.zeros((12))
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
                    contact_normal = f_contact[7]
                    contact_force_v = f_contact[9] * contact_normal[2] / np.linalg.norm(contact_normal)
                    contact_force_l = f_contact[9] * np.linalg.norm(contact_normal[0:1]) / np.linalg.norm(
                        contact_normal)
                    # print(contact_normal)
                    if link_index >= 0:
                        link_name = (p.getJointInfo(robot, link_index)[12]).decode()
                    else:
                        link_name = 'base'

        # result.append((link_name, f_contact[9]))
        result[2 * j] =  phase[j] *contact_force_v
        result[2 * j + 1] = contact_force_l
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


def __get_joints_torque():
        joint_torque = np.zeros(18)
        for j in range(18):
            joint_torque[j] = p.getJointState(robot, j)[3]
        return joint_torque



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
        reward_buff[count]= (new_y - old_y) / dt
        r_l_sum=0
        for index_i in range(count):
            r_l_sum+=reward_buff[index_i]
        r_linear_forward=r_l_sum/(count+1)

    else:
        for index_i in range(buffer_num-1):
            reward_buff[index_i]=reward_buff[index_i+1]
        reward_buff[buffer_num-1]=(new_y - old_y) / dt
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
    
    w_linear_forward = 500
    w_lateral_move = 80
    w_instablity = 8
    w_collision = 0.2
    w_torque = 1  
    reward2 = np.array([min(w_linear_forward * r_linear_forward,52), w_lateral_move * r_lateral_move, w_instablity * r_instablity,
                        max(w_collision * r_collision, -20), max(w_torque * r_torque, -20)])
    
    reward_all=sum(reward2)
    if new_y>0.8:
        print("y",new_y)
        reward_all+=10
    #else:
        #if new_y<0.5:
            #print("y",new_y)
            #reward_all-=20

    print("reward",reward_all," :",reward2)
    return reward2

p.connect(p.GUI)  # 连接到仿真服务器
p.setGravity(0, 0, -9.8)  # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath())  # 设置pybullet_data的文件路径

# 加载地面
floor = p.loadURDF("plane.urdf")

# mini cheetah的初始位置
startPos = [0, 0, 0.1]

# 加载urdf文件
robot = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos)

box = []
# rewards buffer
buffer_num=240
buffer_num_x=90
buffer_num_c=90
buffer_num_t=90
reward_buff=np.zeros(buffer_num)
reward_buff_x=np.zeros(buffer_num_x)
reward_buff_c=np.zeros(buffer_num_c)
reward_buff_t=np.zeros(buffer_num_t)

for j in range(15):
    x_ran = random.uniform(-0.4, 0.4)
    y_ran = random.uniform(0, 0.5)
    Pos = [x_ran, y_ran, 0.05]
    box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos))
Pos = [0.25, 0.4, 0.2]
# box1=p.loadURDF("box_80_60_30/box2.urdf",Pos)
box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos))
Pos2 = [0.4, 0.7, 0.2]
# box2=p.loadURDF("box_80_60_30/box2.urdf",Pos2)
box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos2))
Pos3 = [-2.5, 0.8, 0.2]
# box3=p.loadURDF("box_80_60_30/box2.urdf",Pos3)
box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos3))
box3=p.loadURDF("box_40_30_20/box1.urdf",Pos3)

Pos_slope=[-1,-0.1,0]
Ori=[0.0, 0.0, 0.7071067811865475, 0.7071067811865476]
#slope=p.loadURDF("slope_20/urdf/slope_20.urdf", Pos_slope,Ori,useFixedBase=True)
#box.append(slope)
Pos_slope=[-1,-0.1,0]
Ori=[0.0, 0.0, 0.7071067811865475, 0.7071067811865476]
#slope=p.loadURDF("slope_20/urdf/slope_20.urdf", Pos_slope,Ori,useFixedBase=True)
box.append(floor)
'''
for j in range(18):
    p.changeDynamics(robot,j,lateralFriction=1)
    dyn = p.getDynamicsInfo(robot,j)
    f_friction_lateral=dyn[1]
    print(f_friction_lateral,j)
print("box")
for j in range(14):
    p.changeDynamics(box[j], -1, lateralFriction=1)
    dyn = p.getDynamicsInfo(box[j],-1)
    f_friction_lateral=dyn[1]
    print(f_friction_lateral,j)
'''
# 获取关节数量
numJoints = p.getNumJoints(robot)
# 设置机器人的视觉效果，这里参数-1指的是机器人的base link。
# 我们可以通过rgba值改变机器人相关link的颜色
p.changeVisualShape(robot, -1, rgbaColor=[1, 1, 1, 1])

cf_base = 0.3  # 右边的零位状态　左边增加负号
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

pos1=np.reshape(pos,(6,3))
set_position(pos1)
p.stepSimulation()
for j in range(240 * 50):
    p.stepSimulation()

tf = 10
start_T = time()
zz_n = cpgs.two_layer_out_all(zz0, 0, tf)
t_eval = np.arange(0.0, tf, dt)
end_T = time()
print(end_T - start_T)

theta_f = np.zeros((6, 3))
l1_bc = []
l1_cf = []
l1_ft = []
r3_bc = []
r3_cf = []
r3_ft = []
T = []

count = 0
contacts = np.zeros((1, 12))
T = [0]
theta1_n = np.zeros((1, 6, 3))
phase1_n = np.zeros((1, 6))
reward_n=np.zeros((1, 5))
reward_n_all=[0]
for t in t_eval:
    zz1 = zz_n[:, :, count]
    cpgs.old_zz = zz1
    phase1 = cpgs.get_phase(zz1)
    phase1_n = np.append(phase1_n, np.resize(phase1, (1, 6)), axis=0)
    theta1 = cpgs.cpg2theta_reshape(zz_n[:, :, count])
    theta1_n = np.append(theta1_n, np.resize(theta1, (1, 6, 3)), axis=0)
    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    set_position(theta1)
    # print(phase1)
    # print(t)
    p.stepSimulation()
    count += 1
    location, _ = p.getBasePositionAndOrientation(robot)
    #p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0,
                                 #cameraPitch=-80, cameraTargetPosition=location)
    obser = __get_observation()
    # print(obser)
    contact_result = get_contactPoints(p)

    contacts = np.append(contacts, np.resize(contact_result, (1, 12)), axis=0)
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
    plt.subplot(6, 2, 2*j + 1)
    plt.plot(T, contacts[:, 2*j],'-r',T,phase1_n[:,j],'-b')
    plt.subplot(6, 2, 2*j + 2)
    plt.plot(T, contacts[:, 2 * j+1], '-r', T, phase1_n[:, j], '-b')


plt.show()
'''

plt.Figure()
for j in range(5):
    plt.subplot(3, 2, j + 1)
    plt.plot(T, reward_n[:, j])


plt.subplot(3, 2, 6)
plt.plot(T, reward_n_all)
plt.show()