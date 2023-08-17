import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import math
from CPGs import *

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


p.connect(p.GUI)  # 连接到仿真服务器
p.setGravity(0, 0, -9.8)  # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath())  # 设置pybullet_data的文件路径

# 加载地面
floor = p.loadURDF("plane.urdf")

# mini cheetah的初始位置
startPos = [0, 0, 0.5]

# 加载urdf文件
robot = p.loadURDF("hexapod_23/hexapod_23.urdf", startPos)

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
log=1

dt = 1.0 / 240.0
# theta_phase = [[0, 0.5, 0.25, 0, 0.5, 0.75],[-0.5, 0, -0.25, -0.5, 0, 0.25],
#                [-0.25, 0.25, 0, -0.25, 0.25, 0.5],[0, 0.5, 0.25, 0, 0.5, 0.75],
#                [-0.5, 0, -0.25, -0.5, 0, 0.25],[-0.75, -0.25, -0.5, -0.75, -0.25, 0]]
PH = [0, 1/3, 2/3, 2/3, 0, 1/3]
theta_phase = [[0, 0.5, 0.25, 0, 0.5, 0.75],[-0.5, 0, -0.25, -0.5, 0, 0.25],
               [-0.25, 0.25, 0, -0.25, 0.25, 0.5],[0, 0.5, 0.25, 0, 0.5, 0.75],
               [-0.5, 0, -0.25, -0.5, 0, 0.25],[-0.75, -0.25, -0.5, -0.75, -0.25, 0]]
for j in range(6):
    for i in range(6):
        theta_phase[i][j]=PH[i]-PH[j]

b=0.65

# 计算初始位置并且设置
z0 = np.ones(12) * 0.5
cpgs = CPGs_O(theta_phase, dt,b)
zz0 = cpgs.cpg_begin_all(z0)
theta0 = cpgs.cpg2theta(zz0)
set_position(theta0)
p.stepSimulation()
for j in range(240 * 1):
    p.stepSimulation()

tf=15
zz_n = cpgs.two_layer_out_all(zz0, 0, tf)
t_eval = np.arange(0.0, tf, dt)

theta_f = np.zeros((6, 3))
l1_bc = []
l1_cf = []
l1_ft = []
r3_bc = []
r3_cf = []
r3_ft = []
T = []
if log:
    logID=p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4,fileName="log/robotmove.mp4")
count=0
print("start")
for t in t_eval:

    print(t)
    zz1=zz_n[:,:,count]
    theta1=cpgs.cpg2theta(zz_n[:,:,count])
    set_position(theta1)
    p.stepSimulation()
    count += 1
    location, _ = p.getBasePositionAndOrientation(robot)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                 cameraPitch=-40, cameraTargetPosition=location)
if log:
    p.stopStateLogging(logID)
plt.Figure()
plt.subplot(3, 2, 1)
plt.plot(t_eval, zz_n[0,0,:])
plt.subplot(3, 2, 3)
plt.plot(t_eval, zz_n[0,2,:])
plt.subplot(3, 2, 5)
plt.plot(t_eval, zz_n[0,4,:])
plt.subplot(3, 2, 2)
plt.plot(t_eval, zz_n[5,0,:])
plt.subplot(3, 2, 4)
plt.plot(t_eval, zz_n[5,2,:])
plt.subplot(3, 2, 6)
plt.plot(t_eval, zz_n[5,4,:])
plt.show()
