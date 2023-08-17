from cProfile import label
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import math
from CPGs import *
import json
load_stairs=0

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
    for j in range(6):
        foot_contacts = p.getContactPoints(bodyA=robot, bodyB=floor, linkIndexA=3 * j + 2)
        if not len(foot_contacts):
            link_name = (p.getJointInfo(robot, 3 * j + 2)[12]).decode()
            contact_force = 0

        else:
            for f_contact in foot_contacts:
                link_index = f_contact[3]
                if link_index >= 0:
                    link_name = (p.getJointInfo(robot, link_index)[12]).decode()
                else:
                    link_name = 'base'
                contact_force = f_contact[9]

        # result.append((link_name, f_contact[9]))
        result[j] = contact_force
    return result


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
    phase=cpgs.get_phase(cpgs.old_zz)

    observation=np.append(robot_joint_positions,contact_forces)
    observation = np.append( observation,baseOri)
    observation=np.append(observation,phase)
    return observation


p.connect(p.GUI)  # 连接到仿真服务器
p.setGravity(0, 0, -9.8)  # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath())  # 设置pybullet_data的文件路径

# 加载地面
floor = p.loadURDF("plane.urdf")
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
# mini cheetah的初始位置
startPos = [0, 0, 0.5]

# 加载urdf文件
robot = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos)
Pos = [-0.25, 0.4, 0.2]
box1 = p.loadURDF("box_80_60_30/box2.urdf", Pos,useFixedBase=True)
Pos2 = [-0.4, 0.7, 0.2]
box2 = p.loadURDF("box_80_60_30/box2.urdf", Pos2,useFixedBase=True)
Pos3 = [-0.5, 0.8, 0.2]
box3 = p.loadURDF("box_80_60_30/box2.urdf", Pos3,useFixedBase=True)


textureId = -1

useProgrammatic = 0
useTerrainFromPNG = 1
useDeepLocoCSV = 2
updateHeightfield = False

heightfieldSource = 3
import random
random.seed(10)

heightPerturbationRange = 0.025




if heightfieldSource==useDeepLocoCSV:
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,0.2],fileName = "heightmaps/ground2.txt", heightfieldTextureScaling=128)
  floor  = p.createMultiBody(0, terrainShape)
  p.resetBasePositionAndOrientation(floor,[0,0,0], [0,0,0,1])

if heightfieldSource==useTerrainFromPNG:
  terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD, meshScale=[.1,.1,0.1],fileName = "heightmaps/Maze.png")
  textureId = p.loadTexture("heightmaps/gimp_overlay_out.png")
  floor  = p.createMultiBody(0, terrainShape)
  p.changeVisualShape(floor, -1, textureUniqueId = textureId)
 
 
p.changeVisualShape(floor, -1, rgbaColor=[0.75,0.75,0.75,1],textureUniqueId = textureId)
#p.changeVisualShape(floor, -1, textureUniqueId = textureId)


if load_stairs:
    height=0.01 
    length=0.2
    start_y=5.2
    base_position=[0,start_y,0.025]
    colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                    halfExtents=[5, 5, height])
    
    box=p.createMultiBody(0, colBoxId,-1,base_position)
    for i in range(10):
        base_position=[0,start_y+length*i,height+height*2*i]
        box=p.createMultiBody(0, colBoxId,-1,base_position)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

# 获取关节数量
numJoints = p.getNumJoints(robot)
# 设置机器人的视觉效果，这里参数-1指的是机器人的base link。
# 我们可以通过rgba值改变机器人相关link的颜色
p.changeVisualShape(robot, -1, rgbaColor=[1, 1, 1, 1])

cf_base = 0.3  # 右边的零位状态　左边增加负号
ft_base = -0.3

pos = [0, -cf_base, -ft_base, 0, -cf_base, -ft_base, 0, -cf_base, -ft_base,
       0, cf_base, ft_base, 0, cf_base, ft_base, 0, cf_base, ft_base]
pos1=np.reshape(pos,(6,3))
set_position(pos1)
for j in range(240 * 1):
    p.stepSimulation()

p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                             cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
# 设置每个link的颜色，并且设置初始关节角度
for j in range(numJoints):
    force = 30
    p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, pos[j])

# 　调整至零位
dt = 1. / 50.
p.setTimeStep(dt)

p.stepSimulation()


class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


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

# 计算初始位置并且设置

z0 = np.ones(12) * 0.5
cpgs = CPGs_O(theta_phase, dt, 0.5)


zz0 = cpgs.cpg_begin_all(z0)
theta0 = cpgs.cpg2theta(zz0)


data = {'joint_angles': theta0, 'CPG_output': zz0}
data_json = json.dumps(data, cls=NumpyArrayEncoder)
print(f'data_json:{data_json}')

# 写入json文件
with open('initial_no.json', 'w') as f:
    json.dump(data, f, cls=NumpyArrayEncoder)


with open('initial_no.json', 'r') as f:
    data_read = json.load(f)
    theta0 = np.asarray(data_read['joint_angles'])
    zz0= np.asarray(data_read['CPG_output'])

set_position(theta0)
p.stepSimulation()
for j in range(240 * 1):
    p.stepSimulation()



tf = 35
start_T = time()
zz_n = cpgs.two_layer_out_all(zz0, 0, tf)
t_eval = np.arange(0.0, tf, dt)
end_T = time()
print(end_T - start_T)


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
phase1_n = np.zeros((1, 6))
location, _ = p.getBasePositionAndOrientation(robot)
print("location :",location)
for t in t_eval:
    zz1 = zz_n[:, :, count]
    cpgs.old_zz=zz1
    phase1 = cpgs.get_phase(zz1)
    phase1_n = np.append(phase1_n, np.resize(phase1, (1, 6)), axis=0)
    theta1 = cpgs.cpg2theta_reshape(zz_n[:, :, count])
    theta1_n = np.append(theta1_n, np.resize(theta1, (1, 6, 3)), axis=0)
    set_position(theta1)
    # print(phase1)
    p.stepSimulation()
    count += 1
    location, _ = p.getBasePositionAndOrientation(robot)
    #p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                 #cameraPitch=-40, cameraTargetPosition=location)
    contact_result = get_contactPoints(p)

    contacts = np.append(contacts, np.resize(contact_result, (1, 6)), axis=0)
    T.append(t)
    obseration1=__get_observation()
    #print(obseration1)
'''
print("location :",location)
plt.Figure()
plt.subplot(3, 3, 1)
plt.plot(t_eval, zz_n[0, 0, :])
plt.subplot(3, 3, 3)
plt.plot(t_eval, zz_n[0, 2, :])
plt.subplot(3, 3, 5)
plt.plot(t_eval, zz_n[0, 4, :])
plt.subplot(3, 3, 2)
plt.plot(t_eval, zz_n[5, 0, :])
plt.subplot(3, 3, 4)
plt.plot(t_eval, zz_n[5, 2, :])
plt.subplot(3, 3, 6)
plt.plot(t_eval, zz_n[5, 4, :])
plt.subplot(3, 3, 7)
plt.plot(T, phase1_n[:, 0])
plt.show()

plt.Figure()

plt.plot(t_eval, zz_n[0, 0, :],'r',label='x_00')


plt.plot(t_eval, zz_n[1, 0, :],'b',label='x_10')

plt.plot(t_eval, zz_n[2, 0, :],'c',label='x_20')

plt.plot(t_eval, zz_n[3, 0, :],'r--',label='x_30')

plt.plot(t_eval, zz_n[4, 0, :],'g--',label='x_40')

plt.plot(t_eval, zz_n[5, 0, :],'--',label='x_50')
plt.legend()
plt.xlabel("time(s)")
plt.ylabel("amplitude")

plt.show()

plt.Figure()

plt.plot(t_eval, zz_n[0, 0, :],'r',label='x_00')


plt.plot(t_eval, zz_n[0, 2, :]*0.5,'b',label='x_01')

plt.plot(t_eval, zz_n[0, 4, :]*0.5,'c',label='x_02')


plt.legend()
plt.xlabel("time(s)")
plt.ylabel("amplitude")

plt.show()


plt.Figure()
for j in range(3):
    plt.subplot(3, 1, j + 1)
    plt.plot(T, theta1_n[:, 0, j])
plt.show()

plt.Figure()
for j in range(6):
    plt.subplot(2, 3, j + 1)
    plt.plot(T, contacts[:, j])
plt.show()
'''
