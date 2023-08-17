import gym
from gym import error, spaces, utils
from gym.utils import seeding
import os
import pybullet as p
import pybullet_data as pd
import math
import numpy as np
import random
from CPGs import *
import json
from sys import path
path.append("../../")



class HexaEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    " 真实机器人agent 的顺序为右1 左1 右2 左2.... 0 1 2 3 4 5"
    "目前先用 原来的urdf 模块 左边 01 2 右边 3 4 5 "

    def __init__(self):
        #self._physics_client_id =p.connect(p.GRAPHICS_SERVER_TCP,"10.13.23.175" )  # 连接到仿真服务器
        self._physics_client_id =p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pd.getDataPath())  # 设置pybullet_data的文件路径
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                     cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
        self.action_space = spaces.Box(np.array([-1] * 12), np.array([1] * 12))
        self.observation_space = spaces.Box(np.array([-150] * 33), np.array([150] * 33))
        self.count_num=0
        self.agent_num=6
        p.setGravity(0, 0, -9.8)
        with open('initial_no.json', 'r') as f:
            data_read = json.load(f)
            self.theta0 = np.asarray(data_read['joint_angles'])
            self.zz0 = np.asarray(data_read['CPG_output'])

        # angles,GRF,IMU,phase

    def __set_position(self,theta):
        for j in range(6):
            max_force=4.8
            p.setJointMotorControl2(self.robot, 3 * j, p.POSITION_CONTROL, theta[j, 0],force=max_force)
            p.setJointMotorControl2(self.robot, 3 * j + 1, p.POSITION_CONTROL, theta[j, 1],force=max_force)
            p.setJointMotorControl2(self.robot, 3 * j + 2, p.POSITION_CONTROL, theta[j, 2],force=max_force)


    def __get_observation(self):
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in!")
        # IMU的数据
        basePos, baseOri = p.getBasePositionAndOrientation(self.robot, physicsClientId=self._physics_client_id)
        (x, y, z, w) = baseOri
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        IMU = np.array([roll, pitch, yaw])

        observation_temp=[]
        phase = self.cpgs.get_phase(self.cpgs.old_zz)

        for agent_index in range(self.agent_num):
            JointIndicies_agenti=[3*agent_index,3*agent_index+1,3*agent_index+2]
            robot_joints_states_agenti = p.getJointStates(self.robot, jointIndices=JointIndicies_agenti,
                                                   physicsClientId=self._physics_client_id)
            robot_joint_positions_agenti = np.array([x[0] for x in robot_joints_states_agenti])
            robot_joint_Torque_agenti = np.array([x[3] for x in robot_joints_states_agenti])

            # 竖直方向的接触力
            foot_contacts = []
            contact_force_v = 0
            contact_force_l = 0
            for body_1 in self.box:
                foot_contact = p.getContactPoints(bodyA=self.robot, bodyB=body_1, linkIndexA=3 * agent_index + 2,
                                                  physicsClientId=self._physics_client_id)
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
            observation_agenti = np.append(robot_joint_positions_agenti, robot_joint_Torque_agenti)
            observation_agenti = np.append(observation_agenti,phase[agent_index]*contact_force_v)
            observation_temp.append(observation_agenti)
        observation = np.array(np.vstack(observation_temp)).transpose()
        return observation




    def __get_joints_torque(self):
        joint_torque = np.zeros(18)
        for j in range(18):
            joint_torque[j] = p.getJointState(self.robot, j)[3]
        return joint_torque





    def step(self, action):

        # 暂定1x18的输入
        assert isinstance(action, list) or isinstance(action, np.ndarray)
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in!")
        # 初始状态
        p.setGravity(0, 0, -9.8,physicsClientId=self._physics_client_id)
        #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)

        basePos, baseOri = p.getBasePositionAndOrientation(self.robot,physicsClientId=self._physics_client_id)
        (old_x,old_y,old_z)=basePos
        '''
        obsevation_0=self.__get_observation()
        contact_force0 = np.zeros(12)
        joint_torques = self.__get_joints_torque()
        for j in range(12):
            if obsevation_0[18 + j] >= 0:
                contact_force0[j] =  obsevation_0[18 + j]
        r_linear_forward = 0+ old_y * self.dt * 50
        # 　惩罚侧向的偏移
        r_lateral_move = 0- abs(old_x) * self.dt * 20
        r_instablity =  -0.3 * abs(sum(obsevation_0[30:33])) - 7 * abs(obsevation_0[-1])
        # 惩罚机器人与障碍物的碰撞，以及没有及时抬脚,惩罚摇摆相位出现力反馈
        r_collision = -np.linalg.norm(contact_force0)
        r_torque = -np.linalg.norm(joint_torques)

        w_linear_forward = 40
        w_lateral_move = 10
        w_instablity = 5
        w_collision = 0.5
        w_torque = 0.5
        reward2 = np.array(
            [w_linear_forward * r_linear_forward, w_lateral_move * r_lateral_move, w_instablity * r_instablity,
             max(w_collision * r_collision, -8), max(w_torque * r_torque, -10)])
        reward = sum(reward2)
        print(reward)
        if reward<-2:
            flag=1
        else:
            flag=0
        '''
        # 仿真一步
        d_theta=np.zeros((6,3))
        #print("action: ",action)

        #默认输出归一化后到-1 到1
        for j in range(6):
            d_theta[j,0]=action[2*j]
            d_theta[j,1] =action[2 * j+1]
            d_theta[j,2] =action[2 * j+1]

        Xmin = -1
        Xmax = 1
        # 将数据映射到[-1,1]区间 即a=-1，b=1
        a = -1
        b = 1
        #d_theta = a + (b - a) / (Xmax - Xmin) * (d_theta - Xmin)
        # 获取实时反馈
        '''
        theta_f = np.zeros((6, 3))
        for j in range(6):
            state_feedback = p.getJointState(self.robot, 3 * j)
            theta_f[j, 0] = state_feedback[0]  # 反馈
            state_feedback = p.getJointState(self.robot, 3 * j + 1)
            theta_f[j, 1] = state_feedback[0]  # 反馈
            state_feedback = p.getJointState(self.robot, 3 * j + 2)
            theta_f[j, 2] = state_feedback[0]  # 反馈
        d_theta = np.clip(d_theta, -0.1, 0.1)
        zz0 = self.cpgs.theta2cpg(theta_f, self.cpgs.old_zz)
        self.cpgs.old_zz=zz0
        '''
        zz1=self.cpgs.two_layer_out(self.cpgs.old_zz,self.current_T,self.current_T+(1+0.8)*self.dt)
        self.current_T+=self.dt
        self.cpgs.old_zz=zz1
        theta1 = self.cpgs.cpg2theta_reshape(zz1)
        theta=d_theta+theta1
        # print(flag*d_theta)
        # 对角度值进行裁剪
        limit=0.75
        theta=np.clip(theta,-limit,limit)
        self.__set_position(theta)
        p.stepSimulation()
        self.current_T+=self.dt
        #状态


        state=self.__get_observation()
        contact_force=np.zeros(12)

        contact_force=[state[6,index] for index in range(6) if state[6,index]>0]

        basePos, baseOri = p.getBasePositionAndOrientation(self.robot, physicsClientId=self._physics_client_id)
        (new_x, new_y, new_z) = basePos
        # print(new_z)
        joint_torques = self.__get_joints_torque()
        # 奖励
        r_linear_forward = pow((new_y - old_y) / self.dt, 2) + new_y * self.dt * 50
        # r_linear_forward=pow(new_y,2)
        # 　惩罚侧向的偏移
        r_lateral_move = -abs((new_x - old_x) / self.dt) - abs(new_x) * self.dt * 20
        # 惩罚不稳定性
        #r_instablity = -0.3 * abs(sum(state[30:33])) - 7 * abs(state[-1])
        r_instablity=0
        # 惩罚机器人与障碍物的碰撞，以及没有及时抬脚,惩罚摇摆相位出现力反馈
        r_collision = -np.linalg.norm(contact_force)
        r_torque = -np.linalg.norm(joint_torques)
        # print(f'linear_forward:{r_linear_forward}',f' r_lateral_move:{ r_lateral_move}')
        # print(f'r_instablity:{r_instablity}', f'r_collision:{r_collision}')
        reward1 = np.array([r_linear_forward, r_lateral_move, r_instablity, r_collision])
        w_linear_forward = 40
        w_lateral_move = 10
        w_instablity = 5
        w_collision = 0.5
        w_torque = 0.5
        reward2 = np.array(
            [w_linear_forward * r_linear_forward, w_lateral_move * r_lateral_move, w_instablity * r_instablity,
             max(w_collision * r_collision, -8), max(w_torque * r_torque, -10)])
        reward = sum(reward2)

        #print(reward)

        # 判断是否结束
        if self.current_T>15:
            done=True
        else:
            done=False
        #debug
        info={"state":state,"basePOs":basePos}
        return state,reward,done,info










    def reset(self):
        p.setAdditionalSearchPath(pd.getDataPath())
        p.resetSimulation(physicsClientId=self._physics_client_id)
        p.setGravity(0, 0, -9.8, physicsClientId=self._physics_client_id)
        startPos = [0, 0, 0.2]
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                     cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
        # load robot plane box
        self.plane = p.loadURDF("plane.urdf", physicsClientId=self._physics_client_id)
        self.robot = p.loadURDF("hexapod_23/hexapod_23.urdf", startPos, physicsClientId=self._physics_client_id)

        # 除了机器人外
        self.box = []
        for j in range(1):
            x_ran = random.uniform(-1, 0)
            y_ran = random.uniform(0, 3)
            Pos = [x_ran, y_ran, 0.2]
            # self.box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos))
        Pos = [-0.23, 0.15, 0.2]
        self.box.append(p.loadURDF("box_80_60_25/box.urdf", Pos))

        Pos2 = [-0.25, 0.4, 0.2]
        self.box.append(p.loadURDF("box_80_60_25/box.urdf", Pos2))
        Pos3 = [-0.15, 0.5, 0.2]
        self.box.append(p.loadURDF("box_80_60_25/box.urdf", Pos3))

        self.box.append(self.plane)
        # 三足步态
        dt = 1. / 240.
        self.dt=dt
        p.setTimeStep(dt,physicsClientId=self._physics_client_id)
        theta_phase = [[0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0], [0, 0.5, 0, 0.5, 0, 0.5],
                       [-0.5, 0, -0.5, 0, -0.5, 0],
                       [0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0]]
        self.cpgs = CPGs_O(theta_phase, dt, 0.5)
        self.__set_position(self.theta0)
        for j in range(240):
            p.stepSimulation()
        self.current_T=0
        self.cpgs.old_zz=self.zz0
        '''

        for j in range(18):
            p.changeDynamics(self.robot, j, lateralFriction=10)
            dyn = p.getDynamicsInfo(self.robot, j)
            f_friction_lateral = dyn[1]
            # print(f_friction_lateral, j)
        print("box")
        for j in range(4):
            p.changeDynamics(self.box[j], -1, lateralFriction=10)
            dyn = p.getDynamicsInfo(self.box[j], -1)
            f_friction_lateral = dyn[1]
            # print(f_friction_lateral, j)
        '''

        return self.__get_observation()


    def render(self, mode='human', close=False):
        p.setGravity(0, 0, -9.8, physicsClientId=self._physics_client_id)

        location, _ = p.getBasePositionAndOrientation(self.robot)
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                     cameraPitch=-40, cameraTargetPosition=location)

    def close(self):
        if self._physics_client_id >= 0:
            p.disconnect()
        self._physics_client_id = -1

