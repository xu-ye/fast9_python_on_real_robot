import pybullet as p
import pybullet_data as pd
import matplotlib.pyplot as plt
import numpy as np
import time
from math import *
import math

p.connect(p.GUI)  # 连接到仿真服务器
p.setGravity(0, 0, -9.8)  # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath())  # 设置pybullet_data的文件路径

# 加载地面
floor = p.loadURDF("plane.urdf")

# mini cheetah的初始位置
startPos = [0, 0, 0.5]

# 加载urdf文件
robot = p.loadURDF("hexapod_23/hexapod_23.urdf", startPos)
Pos=[-0.25,0.4,0.2]
#box1=p.loadURDF("box_80_60_30/box2.urdf",Pos)
Pos2=[-0.4,0.7,0.2]
box2=p.loadURDF("box_80_60_30/box2.urdf",Pos2)
Pos3=[-0.5,0.8,0.2]
box3=p.loadURDF("box_80_60_30/box2.urdf",Pos3)

# 获取关节数量
numJoints = p.getNumJoints(robot)
# 设置机器人的视觉效果，这里参数-1指的是机器人的base link。
# 我们可以通过rgba值改变机器人相关link的颜色
p.changeVisualShape(robot, -1, rgbaColor=[1, 1, 1, 1])

cf_base = 0.75  # 右边的零位状态　左边增加负号
ft_base = -0.75

pos = [0, -cf_base, -ft_base, 0, -cf_base, -ft_base, 0, -cf_base, -ft_base,
       0, cf_base, ft_base, 0, cf_base, ft_base, 0, cf_base, ft_base]
# 设置每个link的颜色，并且设置初始关节角度
for j in range(numJoints):
    p.changeVisualShape(robot, j, rgbaColor=[1, 1, 1, 1])
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                 cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
    joint_info = p.getJointInfo(robot, j)
    joint_name = joint_info[1]
    print(joint_name)
    print(j)
    print(pos[j])
    force = 30
    p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, pos[j])
dt = 1. / 240.
p.setTimeStep(dt)
for j in range(240 * 1):
    p.stepSimulation()

p.stepSimulation()
state_t = 0
amp = 0.35
amp2 = 0.15
fre = 0.5
df = 0.3
PH = [0, 0.5, 0, 0.5, 0, 0.5]

t = [[0]]
position = [[0]]
velocity = [[0]]
force_z = [[0]]
time=0

for j in range(18):
    position.append([0])
    velocity.append([0])
    force_z.append([0])
    t.append([0])


def get_contactPoints():
    result = np.zeros((6))
    for j in range(6):
        foot_contacts = p.getContactPoints(bodyA=robot, bodyB=floor,linkIndexA=3*j+2)
        if  not len(foot_contacts) :
            link_name = (p.getJointInfo(robot, 3*j+2)[12]).decode()
            contact_force=0
        
        else:
            for f_contact in foot_contacts:
                link_index = f_contact[3]
                if link_index >= 0:
                    link_name = (p.getJointInfo(robot, link_index)[12]).decode()
                else:
                    link_name = 'base'
                contact_force=f_contact[9]

        # result.append((link_name, f_contact[9]))
        result[j]=contact_force
    return result

def plt_position():
    if time % 240 == 0:
        # plt.clf()
        for j in range(numJoints):
            state_feedback = p.getJointState(robot, j)
            position[j].append(state_feedback[0])
            velocity[j].append(state_feedback[1])
            force_z[j].append(state_feedback[2][0])
            t[j].append(state_t)
        plt.subplot(3,6,1+1)
        plt.plot(t[1],position[1],"-r")
        plt.plot(state_t,state_feedback[0])

        plt.draw()
        plt.pause(dt)


def get_joints_torque():
    joint_torque=np.zeros(18)
    for j in range(18):
        joint_torque[j]=p.getJointState(robot,j)[3]
    return joint_torque

def __get_observation():

    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    robot_joints_states=p.getJointStates(robot,jointIndices=range(18))
    robot_joint_positions=np.array([x[0] for x in robot_joints_states])
    contact_forces = np.zeros(6)
    for j in range(6):
        foot_contacts = p.getContactPoints(bodyA=robot, bodyB=floor, linkIndexA=3 * j + 2)
        if not len(foot_contacts):
            contact_force = 0
        else:
            for f_contact in foot_contacts:
                contact_force = f_contact[9]
        contact_forces[j] = contact_force

    observation=robot_joint_positions+contact_forces+baseOri
    return observation

# plt.ion()
# plt.figure(1)
# 进行仿真i
contacts=np.zeros((1,6))
joints_torque=np.zeros((1,18))
T=[0]
while 1:
    state_t += dt
    s1 = amp * math.sin(fre * 3.14 * state_t)
    s11 = amp2 * math.sin(fre * 3.14 * state_t + df * 2 * 3.14)
    s2 = amp * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[1])
    s22 = amp2 * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[1] + df * 2 * 3.14)
    s3 = amp * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[2])
    s33 = amp2 * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[2] + df * 2 * 3.14)
    s4 = amp * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[3])
    s44 = amp2 * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[3] + df * 2 * 3.14)
    s5 = amp * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[4])
    s55 = amp2 * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[4] + df * 2 * 3.14)
    s6 = amp * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[5])
    s66 = amp2 * math.sin(fre * 3.14 * state_t + 3.14 * 2 * PH[5] + df * 2 * 3.14)

    # 应用到机器人上 bc
    p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, s1)
    p.setJointMotorControl2(robot, 3, p.POSITION_CONTROL, s2)
    p.setJointMotorControl2(robot, 6, p.POSITION_CONTROL, s3)
    p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, -s4)
    p.setJointMotorControl2(robot, 12, p.POSITION_CONTROL, s5)
    p.setJointMotorControl2(robot, 15, p.POSITION_CONTROL, s6)
    # cf
    p.setJointMotorControl2(robot, 1, p.POSITION_CONTROL, -cf_base + s11)
    p.setJointMotorControl2(robot, 4, p.POSITION_CONTROL, -cf_base + s22)
    p.setJointMotorControl2(robot, 7, p.POSITION_CONTROL, -cf_base + s33)
    p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, cf_base - s44)
    p.setJointMotorControl2(robot, 13, p.POSITION_CONTROL, cf_base - s55)
    p.setJointMotorControl2(robot, 16, p.POSITION_CONTROL, cf_base - s66)

    # ft
    p.setJointMotorControl2(robot, 2, p.POSITION_CONTROL, -ft_base + s11)
    p.setJointMotorControl2(robot, 5, p.POSITION_CONTROL, -ft_base + s22)
    p.setJointMotorControl2(robot, 8, p.POSITION_CONTROL, -ft_base + s33)
    p.setJointMotorControl2(robot, 11, p.POSITION_CONTROL, ft_base - s44)
    p.setJointMotorControl2(robot, 14, p.POSITION_CONTROL, ft_base - s55)
    p.setJointMotorControl2(robot, 17, p.POSITION_CONTROL, ft_base - s66)

    # 相机跟踪
    location, _ = p.getBasePositionAndOrientation(robot)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                 cameraPitch=-40, cameraTargetPosition=location)

    p.stepSimulation()
    # 接触点
    contact_result = get_contactPoints()
    print(contact_result)
    contacts=np.append(contacts,np.resize(contact_result,(1,6)),axis=0)
    joint_torque_1=get_joints_torque()
    joints_torque = np.append(joints_torque, np.resize(joint_torque_1, (1, 18)), axis=0)
    T.append(state_t)
    if state_t>20:
        break

    # 获得反馈　并且绘制图像
    time += 1
    
plt.Figure()
for j in range(6):
    plt.subplot(2, 3, j+1)
    plt.plot(T, joints_torque[:,j])


plt.show()











# time.sleep(dt)
