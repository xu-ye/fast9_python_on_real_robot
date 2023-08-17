from CPGs import *
import pybullet as p
import pybullet_data as pd
from time import time,sleep
import math
import matplotlib.pyplot as plt
import numpy as np
from multiprocessing import Process, Queue

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


def get_angles(theta_q):
    PH = [0, 0.5, 0, 0.5, 0, 0.5]
    dt = 1.0 / 120
    theta_phase = [[0, 0.5, 0.25, 0, 0.5, 0.75], [-0.5, 0, -0.25, -0.5, 0, 0.25],
                   [-0.25, 0.25, 0, -0.25, 0.25, 0.5], [0, 0.5, 0.25, 0, 0.5, 0.75],
                   [-0.5, 0, -0.25, -0.5, 0, 0.25], [-0.75, -0.25, -0.5, -0.75, -0.25, 0]]
    for j in range(6):
        for i in range(6):
            theta_phase[i][j] = PH[i] - PH[j]

    b = 0.65
    # 计算初始位置并且设置

    z0 = np.ones(12) * 0.5
    cpgs = CPGs_O(theta_phase, dt, b)
    zz0 = cpgs.cpg_begin_all(z0)
    theta0 = cpgs.cpg2theta(zz0)
    theta_q.put(theta0)

    current_t=0
    n=240
    theta_n=np.zeros((n,6,3))
    while 1:
        start_T=time()
        zz_n=cpgs.two_layer_out_all(zz0,current_t,current_t+(n+0.8)*cpgs.dt)
        end_T = time()
        print("put current T:", current_t, "last T", end_T - start_T, "now :", time())
        for j in range(n):
            theta_n[j,:,:]=cpgs.cpg2theta(zz_n[:,:,j])
        current_t+=n*dt
        theta_q.put(theta_n)

        zz0=zz_n[:,:,-1]
        if current_t >10:
            break


if __name__ == '__main__':

    theta_que = Queue()
    process_child = Process(target=get_angles, args=(theta_que,))
    process_child.start()

    theta0 = theta_que.get()
    print("first get", theta0)

    sleep(2)
    n=240
    #初始化　加载机器人
    p.connect(p.GUI)  # 连接到仿真服务器
    p.setGravity(0, 0, -9.8)  # 设置重力值
    p.setAdditionalSearchPath(pd.getDataPath())  # 设置pybullet_data的文件路
    # 加载地面
    floor = p.loadURDF("plane.urdf")
    # mini cheetah的初始位置
    startPos = [0, 0, 0.5]
    # 加载urdf文件
    robot = p.loadURDF("hexapod_23/hexapod_23.urdf", startPos)
    # 获取关节数量
    numJoints = p.getNumJoints(robot)
    p.changeVisualShape(robot, -1, rgbaColor=[1, 1, 1, 1])
    cf_base = 0.35  # 右边的零位状态　左边增加负号
    ft_base = -0.35
    pos = [0, -cf_base, -ft_base, 0, -cf_base, -ft_base, 0, -cf_base, -ft_base,
            0, cf_base, ft_base, 0, cf_base, ft_base, 0, cf_base, ft_base]
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
    # 设置每个link的颜色，并且设置初始关节角度
    for j in range(numJoints):
        force = 4.2
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, pos[j])
    # 　调整至零位
    dt = 1. / 200.
    p.setTimeStep(dt)
    p.stepSimulation()

    state_t = 0
    # 创建子进程

    # 初始状态

    set_position(theta0)
    p.stepSimulation()
    for j in range(240 * 1):
        p.stepSimulation()

    time_s=time()
    contacts = np.zeros((1, 6))
    T = [0]
    while 1:

        theta_n=theta_que.get()
        print("get ",state_t,"now",time())
        set_n_position(theta_n,n)
        contact_result = get_contactPoints(p)

        state_t+=n*dt
        if state_t>10:
            break
    print("main  end")
    time_e=time()
    print(time_e-time_s)

    print(("all end"))






