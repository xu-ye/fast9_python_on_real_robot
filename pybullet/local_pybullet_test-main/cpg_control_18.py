import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import math
from CPGs import *





def set_position(theta):
	for j in range(6):
		p.setJointMotorControl2(robot, 3 * j, p.POSITION_CONTROL, theta[j,0])
		p.setJointMotorControl2(robot, 3 * j+1, p.POSITION_CONTROL, theta[j, 1])
		p.setJointMotorControl2(robot, 3 * j+2, p.POSITION_CONTROL, theta[j, 2])



p.connect(p.GUI) # 连接到仿真服务器
p.setGravity(0, 0, -9.8) # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath()) # 设置pybullet_data的文件路径

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


cf_base = 0.75  # 右边的零位状态　左边增加负号
ft_base = -0.75


pos = [0, -cf_base, -ft_base, 0, -cf_base, -ft_base, 0, -cf_base, -ft_base,
	 0, cf_base, ft_base, 0, cf_base, ft_base, 0, cf_base, ft_base ]

p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
							 cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
# 设置每个link的颜色，并且设置初始关节角度
for j in range(numJoints):


	force = 30
	p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, pos[j])

#　调整至零位
dt = 1./240.
p.setTimeStep(dt)


p.stepSimulation()

state_t = 0
amp = 0.35
amp2 = 0.15
fre = 0.5
df = 0.3
PH = [0, 0.5, 0, 0.5, 0, 0.5]
dt=1.0/240.0
theta_phase = [[0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0], [0, 0.5, 0, 0.5, 0, 0.5],
			  [-0.5, 0, -0.5, 0, -0.5, 0],
			  [0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0]]


# 计算初始位置并且设置
z0=np.ones(12)*0.5
cpgs = CPGs_O(theta_phase,dt)
zz0=cpgs.cpg_begin_all(z0)
theta0 = cpgs.cpg2theta(zz0)
set_position(theta0)
p.stepSimulation()
for j in range(240*1):
	p.stepSimulation()


# 插值
def interp(theta_0,theta_1,n):
	theta_n=np.zeros((n,6,3))
	#theta_n=theta_0+(theta_1-theta_0)/n
	for j in range(n):
		theta_n[j,:,:]=theta_0+(theta_1-theta_0)/n*(j+1)

	return theta_n


def setbc_position(theta_b):
	for j in range(6):
		if j != 3:
			p.setJointMotorControl2(robot, 3 * j, p.POSITION_CONTROL, theta_b[j])
		if j == 3:
			p.setJointMotorControl2(robot, 3 * j, p.POSITION_CONTROL, -theta_b[j])

def set_nbc_position(theta_k,n):
	for j in range(n):
		setbc_position(theta_k[j])
		p.stepSimulation()

def set_n_position(theta_k,n):
	for j in range(n):
		set_position(theta_k[j,:,:])
		p.stepSimulation()

current_T = 0
theta_f=np.zeros((6,3))
l1_bc=[]
l1_cf=[]
l1_ft=[]
r3_bc=[]
r3_cf=[]
r3_ft=[]
T=[]

# 进行仿真i
while 1:
	# start_t = time.time()
	n= 10
	for j in range(6):
		state_feedback = p.getJointState(robot, 3*j)
		theta_f[j, 0] = state_feedback[0] # 反馈
		state_feedback = p.getJointState(robot, 3 * j+1)
		theta_f[j, 1] = state_feedback[0]  # 反馈
		state_feedback = p.getJointState(robot, 3 * j+2)
		theta_f[j, 2] = state_feedback[0]  # 反馈

	zz1 = cpgs.theta2cpg(theta_f, zz0) # 新cpg
	zz0 = cpgs.two_layer_out(zz1, current_T, current_T+(n+0.8)*dt)
	current_T += n * dt
	theta_new = cpgs.cpg2theta(zz0)

	#插值运行
	theta_k=interp(theta_f, theta_new, n)
	set_n_position(theta_k,n)

	# print(theta_new)

	location, _ = p.getBasePositionAndOrientation(robot)
	p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
								 cameraPitch=-40, cameraTargetPosition=location)

	# end_t=time.time()
	# last_t=end_t-start_t
	# print(last_t)

	if current_T > 10:
		break
	l1bc_feedback = p.getJointState(robot, 0)
	l1_bc.append(l1bc_feedback[0])
	l1cf_feedback = p.getJointState(robot, 1)
	l1ft_feedback = p.getJointState(robot, 2)
	l1_cf.append(l1cf_feedback[0])
	l1_ft.append(l1ft_feedback[0])
	T.append(current_T)
	r3bc_feedback = p.getJointState(robot, 15)
	r3_bc.append(r3bc_feedback[0])
	r3cf_feedback = p.getJointState(robot, 16)
	r3ft_feedback = p.getJointState(robot, 17)
	r3_cf.append(r3cf_feedback[0])
	r3_ft.append(r3ft_feedback[0])

plt.Figure()
plt.subplot(3,2, 1)
plt.plot(T, l1_bc)
plt.subplot(3,2, 3)
plt.plot(T, l1_cf)
plt.subplot(3,2, 5)
plt.plot(T, l1_ft)
plt.subplot(3,2, 2)
plt.plot(T, r3_bc)
plt.subplot(3,2, 4)
plt.plot(T, r3_cf)
plt.subplot(3,2, 6)
plt.plot(T, r3_ft)
plt.show()
