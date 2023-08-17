import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import math
from CPGs import *

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
for j in range(240*1):
	p.stepSimulation()

p.stepSimulation()



state_t = 0
amp = 0.35
amp2 = 0.15
fre = 0.5
df = 0.3
PH = [0, 0.5, 0, 0.5, 0, 0.5]

#计算初始时
z0=np.ones(12)*0.5
cpgs = CPGs_O(dt)
z0=cpgs.cpg_begin(z0)
theta0 = cpgs.cpg2theta(z0)
for j in range(6):
	p.setJointMotorControl2(robot, 3*j, p.POSITION_CONTROL, theta0[j])
p.stepSimulation()


# 插值
def interp(theta_0,theta_1,n):
	theta_n=np.zeros((n,6))
	#theta_n=theta_0+(theta_1-theta_0)/n
	for j in range(n):
		theta_n[j]=theta_0+(theta_1-theta_0)/n*(j+1)

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


current_T = 0
theta=np.zeros(6)
l1_bc=[]
l1_cf=[]
l1_ft=[]
T=[]

# 进行仿真i
while 1:
	# start_t = time.time()
	n=30
	for j in range(6):
		state_feedback = p.getJointState(robot, 3*j)
		theta[j] = state_feedback[0] # 反馈
		if j==3:
			theta[j] = -state_feedback[0]
	z1 = cpgs.theta2cpg(theta, z0) # 新cpg
	z0 = cpgs.cpg_output(z1, current_T, current_T+n*dt)

	theta_bc = cpgs.cpg2theta(z0)

	#插值运行
	theta_10=interp(theta,theta_bc,n)
	set_nbc_position(theta_10,n)

	print(theta_bc)

	location, _ = p.getBasePositionAndOrientation(robot)
	p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
								 cameraPitch=-40, cameraTargetPosition=location)
	current_T += 10*dt
	# end_t=time.time()
	# last_t=end_t-start_t
	# print(last_t)

	if current_T > 3:
		break
	l1bc_feedback = p.getJointState(robot, 0)
	l1_bc.append(l1bc_feedback[0])
	l1cf_feedback = p.getJointState(robot, 1)
	l1ft_feedback = p.getJointState(robot, 2)
	l1_cf.append(l1cf_feedback[0])
	l1_ft.append(l1ft_feedback[0])
	T.append(state_t)

plt.Figure(1)
plt.subplot(3, 1, 1)
plt.plot(T, l1_bc)
plt.subplot(3, 1, 2)
plt.plot(T, l1_cf)
plt.subplot(3, 1, 3)
plt.plot(T, l1_ft)
plt.show()


