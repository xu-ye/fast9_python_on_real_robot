import gym
from gym import error, spaces, utils
from gym.utils import seeding
import os
import pybullet as p
import pybullet_data as pd
import math
import numpy as np
import random

import json
from sys import path
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
path.append(BASE_DIR)
path.append("/usr/local/anaconda3/envs/MAT/lib/python3.8/site-packages/pybullet_examples/Hexapod_Real/Hexapod_Real/envs")
from CPGs import *
from pdControllerExplicit import*
from pdControllerStable import*
from utils import *
from pybullet_functions import *
from reflex_related import *




class HexapodRealEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    " 真实机器人agent 的顺序为右1 左1 右2 左2.... 0 1 2 3 4 5"
    "目前先用 原来的urdf 模块 左边 01 2 右边 3 4 5 "

    def __init__(self):
        show_GUI=0
        self.load_stairs=0
        self.load_blocks=0
        self.load_slops=0
        self.load_steps=0
        
        self.load_real_steps=1
        
        self.with_cpg=1
        self.with_reflex=1
        self.coef_cf=0.2
        # 初始0.2
        
        self.random_pos=1
        
        self.student=0
        self.ablation_body=0
        
        self.tf=6
        self.print_rewards=0
        
        
        self.large_box=1
        self.h_box=0.02
        self.l_box=0.05
        self.w_box=0.05
        self.with_delay=False
        
        
        self.height=0.01
        self.length=0.15
        
        self.slope_angles=np.pi/180*15
        
        self.prob1=[0.3,0.3,0.25,0.1,0.05]
        
        self.step_height=0.03
        self.random_seed=4
        ## 4是比较容易训练的随机数
        ## 3比较难  5中等
        ## 6相对较难  入口高度差大 但是较为均衡
        ## 8 相对简单
        #print(self.)
        self.change_steps=0
        self.eval_random= 1
        self.count_episode=0
        
        if show_GUI:
            self._physics_client_id =p.connect(p.GRAPHICS_SERVER_TCP,"10.13.23.175")  # 连接到仿真服务器
        #self._physics_client_id =p.connect(p.GRAPHICS_SERVER_TCP,"10.250.7.225")  # 连接到仿真服务器
        #self._physics_client_id =p.connect(p.GRAPHICS_SERVER_TCP,"192.168.156.21")
            #self._physics_client_id =p.connect(p.GUI)
        else:
            self._physics_client_id =p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pd.getDataPath())  # 设置pybullet_data的文件路径
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                     cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
        #self.action_space = spaces.Box(np.array([-1] * 12), np.array([1] * 12)) 
        # action space 应该为同款的6，2  #dtype=np.float32
        self.action_space = spaces.Box(low=np.float32(-1),high=np.float32(1),shape=(6,1))
        self.observation_space = spaces.Box(low=np.float32(-150),high=np.float32(150),shape=(6,11))
        self.share_observation_space=spaces.Box(low=np.float32(-150),high=np.float32(150),shape=(6,117))
        #self.observation_space = spaces.Box(np.array([-150] * 33), np.array([150] * 33))
        self.count_num=0
        self.n_agents=6
        self.agent_num=6
        
        
        self.buffer_num=240
        self.buffer_num_x=90
        self.buffer_num_c=90
        self.buffer_num_t=90
        self.reward_buff=np.zeros(self.buffer_num)
        self.reward_buff_x=np.zeros(self.buffer_num_x)
        self.reward_buff_c=np.zeros(self.buffer_num_c)
        self.reward_buff_t=np.zeros(self.buffer_num_t)
        self.count=0
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pd.getDataPath())
        p.resetSimulation(physicsClientId=self._physics_client_id)
        p.setGravity(0, 0, -9.8, physicsClientId=self._physics_client_id)
        self.startPos = [0,-0.05, 0.1]
        #ori= self.rpy2quaternion(self.slope_angles,0,0)
        self.startOri=self.rpy2quaternion(0,0,0)
        self.startOri_slop=[0.17364817766693033, 0.0, 0.0, 0.984807753012208]
        # load robot plane box
        if show_GUI:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
            self.plane = p.loadURDF("plane.urdf", physicsClientId=self._physics_client_id)
            self.robot = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", self.startPos, physicsClientId=self._physics_client_id)
            #self.robot = p.loadURDF("hexapod_23/hexapod_23.urdf", self.startPos,self.startOri, physicsClientId=self._physics_client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        else:
            self.plane = p.loadURDF("plane.urdf", physicsClientId=self._physics_client_id)
            self.robot = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", self.startPos, physicsClientId=self._physics_client_id)
            #self.robot = p.loadURDF("hexapod_23/hexapod_23.urdf", self.startPos, self.startOri,physicsClientId=self._physics_client_id)

        # 除了机器人外
        
        
        ## 加载servo_client
        self.servo_client=p.connect(p.DIRECT)
        #servo_client=p.connect(p.GUI)
        p.setGravity(0, 0, -9.8,physicsClientId=self.servo_client)  # 设置重力值
        p.setAdditionalSearchPath(pd.getDataPath(),physicsClientId=self.servo_client)  # 设置pybullet_data的文件路径

        # 加载地面
        self.floor_servo = p.loadURDF("plane.urdf",physicsClientId=self.servo_client)
        self.box_servo=[]
        self.box_servo.append(self.floor_servo)

        startPos_servo=[0,0,1]
        # 加载urdf文件
        self.robot_servo = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos_servo,physicsClientId=self.servo_client,useFixedBase=True,)
        dt=1/1000
        p.setTimeStep(dt,physicsClientId=self.servo_client)        
        
        
        with open('/usr/local/anaconda3/envs/hexapod/lib/python3.7/site-packages/pybullet_examples/hexapod_MA/gym-hexapod/initial_no.json', 'r') as f:
            data_read = json.load(f)
            self.theta0 = np.asarray(data_read['joint_angles'])
            self.zz0 = np.asarray(data_read['CPG_output'])
        
        #with open('/usr/local/anaconda3/envs/hexapod/lib/python3.7/site-packages/pybullet_examples/hexapod_MA/gym-hexapod/force_real7.json', 'r') as f:
        #with open('/usr/local/anaconda3/envs/hexapod/lib/python3.7/site-packages/pybullet_examples/hexapod_MA/gym-hexapod/force_record2.json', 'r') as f:
        with open('/usr/local/anaconda3/envs/MAT/lib/python3.8/site-packages/pybullet_examples/Hexapod_Real/Hexapod_Real/envs/force_real17.json', 'r') as f:
            self.data_read = json.load(f)
            self.force_r = np.asarray(self.data_read['contact_force'])
            self.phase_r = np.asarray(self.data_read['phase'])
            self.cpg_r = np.asarray(self.data_read['cpg'])
            self.theta_r = np.asarray(self.data_read['theta'])
            self.theta_ini_r=np.asarray(self.data_read['theta_ini'])
            self.ini_index_r=np.asarray(self.data_read['ini_index'])
            self.end_pos=np.asarray(self.data_read['end_pos'])
            
            self.torque_n=np.asarray(self.data_read['torque_n'])
            self.theta_real_n=np.asarray(self.data_read['theta_real_n'])

        
        #self.cpg_index=self.ini_index_r+120
        self.cpg_index=self.ini_index_r[0]
        
        
        self.box = []
        '''
        for j in range(1):
            x_ran = random.uniform(-0.4, 0.4)
            y_ran = random.uniform(-0.1, 1)
            Pos = [x_ran, y_ran, 0.05]
            #self.box.append(p.loadURDF("box_80_60_25/box.urdf", Pos))
        
        x_ran_a0=np.random.randint(low=-10, high=12, size=10+5)*0.03
        np.random.shuffle(x_ran_a0)
        x_ran_a=np.unique(x_ran_a0)[:12]
        np.random.shuffle(x_ran_a)
        y_ran_a0=np.random.randint(low=15, high=50, size=10+5)*0.01
        y_ran_a=np.unique(y_ran_a0)[:12]
        x_a=np.arange(-12,12,1)*0.03
        np.random.shuffle(x_a)
        np.random.shuffle(x_a)
        y_a=np.arange(5,70,3)*0.01
        np.random.shuffle(y_a)
        '''
        if self.load_blocks:
            x_1=[-0.25,-0.1,0.05,0.2,0.35,-0.35,-0.2,-0.05,0.1,0.25]
            np.random.shuffle(x_1)
            np.random.shuffle(x_1)
            np.random.shuffle(x_1)
            #np.random.shuffle(x_1)
            y_1=np.arange(14,34,2)*0.01
            #np.random.shuffle(y_1)  
            y_2=np.arange(34,54,2)*0.01
            
            if self.large_box:
                
                length=0.3
                base_position=[0,4.8,0.025]
                hal_geo=[self.l_box, self.w_box, self.h_box]
                colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                                halfExtents=hal_geo)
                visualID=p.createVisualShape(p.GEOM_BOX,
                                                halfExtents=hal_geo)                                  
                box=p.createMultiBody(0, colBoxId,visualID,base_position)
            
                for j in range(10):
                    Pos = [x_1[j], y_1[j], self.h_box] 
                    box=p.createMultiBody(0, colBoxId,visualID,Pos)
                    self.box.append(box)
                    #self.box.append(p.loadURDF("box_80_60_25/box.urdf", Pos,useFixedBase=True))
                for j in range(10):
                    Pos = [x_1[j], y_2[j], self.h_box]
                    box=p.createMultiBody(0, colBoxId,visualID,Pos)
                    self.box.append(box) 
                    #self.box.append(p.loadURDF("box_80_60_25/box.urdf", Pos,useFixedBase=True))          
            else:
                for j in range(10):
                    Pos = [x_1[j], y_1[j], 0] 
                    
                    self.box.append(p.loadURDF("box_80_60_25/box.urdf", Pos,useFixedBase=True))
                for j in range(10):
                    Pos = [x_1[j], y_2[j], 0]
                    
                    self.box.append(p.loadURDF("box_80_60_25/box.urdf", Pos,useFixedBase=True))    
           
        '''
        self.Pos1 = [-0.25, 0.10, 0.05]
        self.Box1=p.loadURDF("box_80_60_25/box.urdf", self.Pos1)
        self.box.append(self.Box1)

        self.Pos2 = [-0.25, 0.3, 0.05]
        self.Box2=p.loadURDF("box_80_60_25/box.urdf", self.Pos2)
        self.box.append(self.Box2)
        self.Pos3 = [0.23, 0.16, 0.05]
        self.Box3=p.loadURDF("box_80_60_25/box.urdf", self.Pos3)
        self.box.append(self.Box3)
        #Position=np.vstack((Pos,self.Pos2,self.Pos3))
        #info={"position":Position}
        '''
        self.step_width=0.05
        self.height_map=np.zeros((20,20)) 
        if self.load_steps:
            height=0.2 
            length=0.3
            
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            
            hal_geo=[self.step_width, self.step_width, height]
            colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                            halfExtents=hal_geo)
            visualID=p.createVisualShape(p.GEOM_BOX,
                                            halfExtents=hal_geo) 
            
            np.random.seed(self.random_seed)
            x_range=np.arange(-self.step_width*(2*10-1),self.step_width*(2*10+1),self.step_width*2) 
            y_range=np.arange(self.step_width*(2*2-1),self.step_width*(2*22-1),self.step_width*2)
            self.step_start_place=[x_range[0]-self.step_width,y_range[0]-self.step_width]
            self.height_map=np.zeros((20,20)) 
            for i in range(20):
                for j in range(20):
                    if j<10:
                        hei_b=np.random.random()*self.step_height*(2)-height
                    else:
                        hei_b=np.random.random()*self.step_height*(2)-height+self.step_height
                    base_position=[x_range[i],y_range[j],hei_b] 
                    self.height_map[i,j]=hei_b +height  
                    box=p.createMultiBody(0, colBoxId,visualID,base_position)
                    self.box.append(box)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        if self.load_real_steps:
            height=0.2 
            length=0.3
            
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            
            hal_geo=[self.step_width, self.step_width, height]
            colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                            halfExtents=hal_geo)
            visualID=p.createVisualShape(p.GEOM_BOX,
                                            halfExtents=hal_geo) 
            
            np.random.seed(self.random_seed)
            x_range=np.arange(-self.step_width*(2*10-1),self.step_width*(2*10+1),self.step_width*2) 
            y_range=np.arange(self.step_width*(2*2-1),self.step_width*(2*22-1),self.step_width*2)
            self.step_start_place=[x_range[0]-self.step_width,y_range[0]-self.step_width]
            self.height_map=np.zeros((20,20)) 
            for i in range(20):
                for j in range(20):
                    if j<10:
                        height_cm=np.random.choice([2,3,5,8,10],p=[0.33,0.33,0.30,0.03,0.01])
                        hei_b=height_cm*0.01-height
                    else:
                        height_cm=np.random.choice([2,3,5,8,10],p=[0.3,0.25,0.2,0.15,0.1])
                        hei_b=height_cm*0.01-height
                    base_position=[x_range[i],y_range[j],hei_b] 
                    self.height_map[i,j]=hei_b +height  
                    box=p.createMultiBody(0, colBoxId,visualID,base_position)
                    self.box.append(box)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        
        
        #Position=[1,1,1]
        if self.load_slops:
            height=0.05
            length=3
            width=5
            base_position=[0,0.4,0.025]
            hal_geo=[width, width, height]
            colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                            halfExtents=hal_geo)
            visualID=p.createVisualShape(p.GEOM_BOX,
                                            halfExtents=hal_geo) 
            ori= self.rpy2quaternion(self.slope_angles,0,0)                             
            slope=p.createMultiBody(0, colBoxId,visualID,base_position,ori)
            self.Pos_slope=[-1,-1,0]
            self.Ori=[0.0, 0.0, 0.7071067811865475, 0.7071067811865476]
            #self.slope=p.loadURDF("slope_30/urdf/slope_30.urdf", self.Pos_slope,self.Ori,useFixedBase=True)
            #self.slope=p.loadURDF("slope_20/urdf/slope_20.urdf", self.Pos_slope,self.Ori,useFixedBase=True)
            self.box.append(slope)

        self.box.append(self.plane)
        
        if self.load_stairs:
            #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
            #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            textureId=-1
            height=self.height
            length=self.length
            start_y=5.2
            self.height_map=np.zeros(10) 
            base_position=[0,start_y,0.025]
            colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                            halfExtents=[5, 5, height])
            #height=0.05  
            box=p.createMultiBody(0, colBoxId,-1,base_position)
            for i in range(10):
                base_position=[0,start_y+length*i,height+2*height*i]
                box=p.createMultiBody(0, colBoxId,-1,base_position)
                visual1=p.createVisualShape(p.GEOM_BOX,halfExtents=[5, 5, height])
                box=p.createMultiBody(0, colBoxId,visual1,base_position)
                p.changeVisualShape(box, -1, rgbaColor=[1,1,1,1],textureUniqueId = textureId)
                self.box.append(box)
                self.height_map[i]=2*height*(i+1)
            #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
            #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

        
        for j in range(18):
            p.changeDynamics(self.robot,j,lateralFriction=1,physicsClientId=self._physics_client_id)
            dyn = p.getDynamicsInfo(self.robot,j,physicsClientId=self._physics_client_id)
            f_friction_lateral=dyn[1]
            #print(f_friction_lateral,j)
        p.changeDynamics(self.plane, -1, lateralFriction=3,physicsClientId=self._physics_client_id)
        for j in range(1):
            p.changeDynamics(self.box[j], -1, lateralFriction=3,physicsClientId=self._physics_client_id)
            dyn = p.getDynamicsInfo(self.box[j],-1)
            f_friction_lateral=dyn[1]
            #print(f_friction_lateral,j)
        
        # 三足步态
        dt=1/1000
        self.dt=dt
        theta_phase = [[0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0], [0, 0.5, 0, 0.5, 0, 0.5],
               [-0.5, 0, -0.5, 0, -0.5, 0],
               [0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0]]


        self.cpgs = CPGs_O(theta_phase, dt, 0.5)
        
        
        p.setTimeStep(dt,physicsClientId=self._physics_client_id)
        Force = 4.1
        
        p.stepSimulation(physicsClientId=self._physics_client_id)
        self.__set_position(self.theta_ini_r)
        for j in range(120):
            p.stepSimulation(physicsClientId=self._physics_client_id)
        self.current_T=0
        self.cpgs.old_zz=self.cpg_r[:,:,self.cpg_index]
        
        self.reward_buff=np.zeros(self.buffer_num)
        self.reward_buff_x=np.zeros(self.buffer_num_x)
        self.reward_buff_c=np.zeros(self.buffer_num_c)
        self.reward_buff_t=np.zeros(self.buffer_num_t)
        self.count=0
        self.cpg_count=0
        
        
        
        # reflex 相关参数的初始化
        self.traj_error_buf=np.zeros((4,18))
        self.on_reflex=np.zeros(6)
        self.reflex_index=np.ones(6)*(-10)
        self.on_reflex_stance=np.zeros(6)
        self.reflex_index_stance=np.ones(6)*(-10)
        self.stance_step_per_reflex=np.ones(6)*(0)
        self.swing_step_per_reflex=np.ones(6)*(0)
        self.sum_leg_all=[]
        self.reflex_real_all=[]
        self.on_reflex_all=[]
        self.traj_error_all=[]
        self.count_all=[]
        self.csv_rows=[]
        self.coef_real=np.zeros(6)
        
        
        
        self.T=240
        self.T_count=0
        coef=2
        self.coef_stance=1
        self.reflex=np.zeros(6)
        # 计数在每个swing or reflex中的步数
        self.swing_step_count=0
        self.reflex_count=np.zeros(6)
        self.step_count=0
        self.swing_coef=2
        self.stance_coef=20
        self.seed_count=0
        


        # angles,GRF,IMU,phase

    def __set_position(self,theta):
        # theta (6,3) 单位弧度  1=在当前目标下运动10ms
        #0.01647,0.397,8.92
        frictionForce=0.9
        P_gain=0.01647
        D_gain=0.397
        max_speed=8.92
        Force=4.1
        p.setJointMotorControlArray(self.robot, range(18), p.VELOCITY_CONTROL, targetVelocities=[0]*18, forces=[frictionForce]*18)
                    # 直接应用real tau 完全没有按照既定轨迹运行
        theta1=theta.flatten()  
        for tick_num in range(10):         
            for actuator_index in range(18):
                p.setJointMotorControl2(bodyUniqueId=self.robot,jointIndex=actuator_index,controlMode=p.POSITION_CONTROL,targetPosition=theta1[actuator_index],
                positionGain=P_gain,
                velocityGain=D_gain,
                maxVelocity=max_speed,
                force = Force)
            p.stepSimulation(physicsClientId=self._physics_client_id)    

    
    def get_forward_pos(self,pos):
        theta_1=0+pos[0]
        theta_2=-(90-21.36-0)/180*3.1415926-pos[1]
        theta_3=(180-(62.69+0-21.36-15))/180*3.1415926-pos[2]
        
        T_01=np.array([[np.cos(theta_1),-np.sin(theta_1),0,0],[np.sin(theta_1),np.cos(theta_1),0 ,0],[0, 0 ,1, -33.33/1000],[0, 0, 0 ,1]])
        T_12=np.array([[np.cos(theta_2),-np.sin(theta_2),0,42.55/1000],[0,0,1,0],[-np.sin(theta_2),-np.cos(theta_2),0,0],[0,0,0,1]])
        T_23=np.array([[np.cos(theta_3),-np.sin(theta_3),0,64.25/1000],[np.sin(theta_3),np.cos(theta_3),0,0],[0,0,1,0],[0,0,0,1]])
        T_34=np.array([[1,0,0,135.4/1000],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        T_04=T_01@T_12@T_23@T_34
        end_pos=T_04[0:3,3]
        ouler_angles=self.rotationMatrixToEulerAngles(T_04[0:3,0:3])
        return end_pos,ouler_angles

    def rotationMatrixToEulerAngles(self,R) :

        
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
    
    def rpy2quaternion(self,roll, pitch, yaw):
        x=math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
        y=math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
        z=math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
        w=math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
        ori=np.array([x,y,z,w])
        return ori
    
    
    def __get__toruqe(self):
        torque_target_all=[]
        torque_error_all=[]
        for agent_index in range(self.agent_num):
            if agent_index<self.agent_num-1:
                JointIndicies_agenti=[3*agent_index,3*agent_index+1,3*agent_index+2]
                robot_joints_states_agenti = p.getJointStates(self.robot, jointIndices=JointIndicies_agenti,
                                                    physicsClientId=self._physics_client_id)
                robot_joint_positions_agenti = np.array([x[0] for x in robot_joints_states_agenti])
                robot_joint_Torque_agenti = np.array([x[3] for x in robot_joints_states_agenti])
                
                torque_target=np.array(self.torque_n[self.cpg_index,agent_index,:])
                torque_error=np.array(torque_target-robot_joint_Torque_agenti)
                torque_error_all.append(torque_error)
                torque_target_all.append(torque_target)
        torque_error_all=np.array(np.vstack(torque_error_all))
        torque_target_all=np.array(np.vstack(torque_target_all))
        return torque_error_all,torque_target_all
        



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
        length_12=0.126
        length_13=0.266
        dz1=-length_12*math.tan(roll)
        dz2=-length_13*math.tan(roll)

        observation_temp=[]
        
        #phase = self.cpgs.get_phase(self.cpgs.old_zz)# -1是支撑相 1是摇摆相
        phase=self.phase_r[self.cpg_index,:]
        theta_old = self.cpgs.cpg2theta_reshape(self.cpgs.old_zz)
        comtact_force_leg=np.zeros(self.agent_num-1)
        foot_z=np.zeros(self.agent_num)

        for agent_index in range(self.agent_num):
            if agent_index<self.agent_num:
                JointIndicies_agenti=[3*agent_index,3*agent_index+1,3*agent_index+2]
                robot_joints_states_agenti = p.getJointStates(self.robot, jointIndices=JointIndicies_agenti,
                                                    physicsClientId=self._physics_client_id)
                robot_joint_positions_agenti = np.array([x[0] for x in robot_joints_states_agenti])
                
               
                #zworld_end_ori=p.getLinkState(self.robot,3*agent_index+2,computeForwardKinematics=True)[0]
                if agent_index>=3:
                    end_pos,end_ori=self.get_forward_pos(-robot_joint_positions_agenti)
                else:
                    end_pos,end_ori=self.get_forward_pos(robot_joint_positions_agenti)
                foot_z[agent_index]=end_pos[2]
                if agent_index%3==1:
                    foot_z[agent_index]=end_pos[2]+dz1
                
                if agent_index%3==2:
                    foot_z[agent_index]=end_pos[2]+dz2
                        
                relative_foot_z=foot_z[agent_index]-foot_z[0]
            
                # 误差的error
                

                # 相位信息
                
                #phase_continus=self.cpgs.old_zz[agent_index,2:4]
                phase_continus=self.cpg_r[agent_index,2:4,self.cpg_index,]
  
                 
                    
                observation_agenti = np.append(robot_joint_positions_agenti, IMU)
                observation_agenti = np.append(observation_agenti,relative_foot_z)
                observation_agenti = np.append(observation_agenti,phase_continus)
                observation_agenti = np.append(observation_agenti,phase[agent_index])
                observation_agenti = np.append(observation_agenti,self.coef_real[agent_index])
                observation_temp.append(observation_agenti)
            
        
                
                
                
                
        observation = np.array(np.vstack(observation_temp),dtype=np.float64)
        return observation
    

    

    def __get_share_observation(self):
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
        phase = self.cpgs.get_phase(self.cpgs.old_zz)
        theta_old = self.cpgs.cpg2theta_reshape(self.cpgs.old_zz)

        for agent_index in range(self.agent_num):
            JointIndicies_agenti=[3*agent_index,3*agent_index+1,3*agent_index+2]
            robot_joints_states_agenti = p.getJointStates(self.robot, jointIndices=JointIndicies_agenti,
                                                   physicsClientId=self._physics_client_id)
            robot_joint_positions_agenti = np.array([x[0] for x in robot_joints_states_agenti])
            robot_joint_Torque_agenti = np.array([x[3] for x in robot_joints_states_agenti])
            world_end_position=p.getLinkState(self.robot,3*agent_index+2,computeForwardKinematics=True)[0]
            world_end_ori=p.getLinkState(self.robot,3*agent_index+2,computeForwardKinematics=True)[0]
        
            # 误差的error
            theta_error=robot_joint_positions_agenti-theta_old[agent_index,:]



            # 相位信息
            phase_continus1=self.cpgs.old_zz[agent_index,2]
            phase_continus2=self.cpgs.old_zz[agent_index,3]
            phase_continus=self.cpgs.old_zz[agent_index,2:4]
            force_sin=self.force_r[abs(self.cpg_r[agent_index,2,:]-phase_continus1)<1e-3,agent_index]
            force_cos=self.force_r[abs(self.cpg_r[agent_index,3,:]-phase_continus2)<1e-3,agent_index]
            force_t1=np.intersect1d(force_cos,force_sin)
            if len(force_t1):
                force_t=force_t1[0]
            else:
                force_t=0
        


            # 竖直方向的接触力
            foot_contacts = []
            contact_force_v = 0
            contact_force_l = 0
            contact_point_num=0
            contact_position=np.zeros(3)
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
                        contact_force_l_now=f_contact[9] * np.linalg.norm(contact_normal[0:1]) / np.linalg.norm(
                            contact_normal)
                        contact_point_num+=1
                        if (abs(contact_force_l_now)<1e-3):
                            contact_position=f_contact[6]
            #if (contact_point_num):
                
            contact_force_diff=contact_force_v-force_t
            #print("contact_force_diff ",contact_force_diff)
            observation_agenti = np.append(robot_joint_positions_agenti, robot_joint_Torque_agenti)
            observation_agenti = np.append(observation_agenti,contact_force_v)
            observation_agenti = np.append(observation_agenti,contact_force_diff)
            
            observation_agenti = np.append(observation_agenti,world_end_position)
            observation_agenti = np.append(observation_agenti,world_end_ori)
            observation_agenti = np.append(observation_agenti,phase_continus)
            observation_agenti = np.append(observation_agenti,theta_error)
            observation_temp.append(observation_agenti)
        observation = np.array(np.vstack(observation_temp),dtype=np.float64).flatten()
        observation_share=np.append(observation,IMU)
        share_obs=[observation_share for j in range(self.agent_num)]
        return share_obs





    def __get_joints_torque(self):
        joint_torque = np.zeros(18)
        for j in range(18):
            joint_torque[j] = p.getJointState(self.robot, j)[3]
        return joint_torque
    
    def __rpy2quaternion(roll, pitch, yaw):
        x=math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
        y=math.sin(pitch/2)**math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
        z=math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
        w=math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
        ori=[x,y,z,w]
        return ori





    def step(self, action):
        
        # action  各条腿的action 是对应的coef
        

        # 暂定6x1的输入
        '''
        for j in range(18):
            p.changeDynamics(self.robot,j,lateralFriction=1,physicsClientId=self._physics_client_id)
            dyn = p.getDynamicsInfo(self.robot,j,physicsClientId=self._physics_client_id)
            f_friction_lateral=dyn[1]
            #print(f_friction_lateral,j)
        p.changeDynamics(self.plane, -1, lateralFriction=3,physicsClientId=self._physics_client_id)
        '''
        assert isinstance(action, list) or isinstance(action, np.ndarray)
        if not hasattr(self, "robot"):
            assert Exception("robot hasn't been loaded in!")
        # 初始状态
        
        if self.cpg_index<239:
            self.cpg_index+=1
        else:
            self.cpg_index=0
            
        self.current_T+=1/240
        
        

        basePos, baseOri = p.getBasePositionAndOrientation(self.robot,physicsClientId=self._physics_client_id)
        (old_x,old_y,old_z)=basePos
        basePos, baseOri = p.getBasePositionAndOrientation(self.robot, physicsClientId=self._physics_client_id)
        (x, y, z, w) = baseOri
        
        
        contact_force0 = np.zeros(12)
        joint_torques = self.__get_joints_torque()
        IMU_data=get_IMU(self.robot)
        (roll ,pitch,yaw)=IMU_data[0:3]/3.1415926*180 # 绕着 xy z轴转动
        #print("roll pitch yaw: ",roll ,pitch,yaw)
        
        position_Read=get_position(self.robot,self._physics_client_id)
        
        
        
        
        # judge contact  reflex 
        phase_now=self.phase_r[self.cpg_index,:]
        last_phase=self.phase_r[self.cpg_index-1]
        #contact_v,contact_l=get_contactPoints(p,self.robot,self.box)
        reflex,sum_leg=judge_reflex(self.traj_error_buf)
        #print("sum_leg",sum_leg)
        reflex_sim=swing_reflex(reflex,phase_now)
        # 判断是不是reflex 基础上的 reflex
        reflex_sim_new=judge_new_reflex(reflex_sim,self.swing_step_per_reflex)
        if reflex_sim.any():
            self.reflex_index[reflex_sim>0]=self.T_count
            self.on_reflex[reflex_sim>0]=1

            # 每有一个新的reflex  swing_step_per_reflex   置1
            #    
        if reflex_sim_new.any():
            self.swing_step_per_reflex[reflex_sim_new>0]=0
        if self.on_reflex.any():
            self.swing_step_per_reflex[self.on_reflex>0]+=1
                
        self.swing_step_per_reflex[self.swing_step_per_reflex>239]=0
            # swing 和stance  交界处 error 清零
        self.swing_step_per_reflex[(last_phase*phase_now)<0]=0
        self.swing_step_count+=1
        if (last_phase*phase_now)[0]<1:
            self.traj_error_buf=np.zeros_like(self.traj_error_buf)
            self.swing_step_count=0
            self.on_reflex=np.zeros_like(self.on_reflex)
            self.reflex_count=np.zeros_like(self.on_reflex)
                
        for i in range(6):# 当进入下一周期后 默认不在reflex范围内
            if (self.T_count-self.reflex_index[i])==1 and self.on_reflex[i]>0:
                self.on_reflex[i]=0
                self.reflex_index[i]=-2
                self.swing_step_per_reflex[i]=0   
        if (self.swing_step_per_reflex==1).any() :
            self.reflex_count[self.swing_step_per_reflex==1]+=1
            for index_i in range(30):
                set_position_multi(position_Read,self.robot_servo,self.servo_client)
            position_servo=get_position(self.robot_servo,self.servo_client)



        # imu reflex   
        imu_reflex=judge_imu_reflex2(IMU_data,phase_now)
        #coef_imu=get_imu_coef(IMU_data,imu_reflex)



        #set coef 对应的coef为65
        # stance  最多整体变化1 
        
        #stance_coef=0
        #coef_delta=0.4
        action_coef=np.zeros(6)
        for coef_index in range(6):
            if phase_now[coef_index]==1:# 摇摆相
                action_coef[coef_index]=(action[coef_index]* self.swing_coef)[0]
            elif phase_now[coef_index]==-1:# 支撑相
                action_coef[coef_index]=(action[coef_index]* self.stance_coef)[0]
        if self.swing_step_count<10:
            coef=1*np.ones(6)
                #coef_stance=1
        else:
            #coef=np.clip(1*np.ones(6)+coef_delta*(reflex_count),0,3)
                #coef_stance=1.5
            coef=np.clip(1*np.ones(6)+action_coef,0,3)
            #print("reflex sim",reflex_sim,"on reflex",self.on_reflex)
        swing_coef_apply=np.zeros(6)   
        for i in range(6):
            if self.swing_step_per_reflex[i]==1:
                swing_coef_apply[i]=coef[i]
                self.last_reflex_coef[i]=swing_coef_apply[i]
            else:
                swing_coef_apply[i]=self.last_reflex_coef[i]
            
                
        
        #print("traj_error_buf",self.traj_error_buf,)
        imu_coef=action_coef+np.ones(6)
        coef_stance=np.zeros(6)
        reflex_stance_sim=np.zeros(6)
        # robot move
        theta_sim=self.theta_r[self.cpg_index]      
        theta_new_sim0=get_reflex_theta_all(theta_sim,swing_coef_apply,coef_stance,reflex_sim,self.on_reflex,reflex_stance_sim,self.on_reflex_stance,phase_now,self.stance_step_per_reflex,self.swing_step_per_reflex)
        # imu reflex
        theta_new_sim=imu_roll_reflex2(imu_reflex,phase_now,theta_new_sim0,imu_coef)   
        set_position_multi(theta_new_sim,self.robot,self._physics_client_id)
        
        # 实际运行的幅值
        
        for i in range(6):
            if phase_now[i]==1:#摇摆相
                self.coef_real[i]=(swing_coef_apply[i]*self.on_reflex[i])/( self.swing_coef+1)
            elif phase_now[i]==-1:#支撑相
                self.coef_real[i]=(imu_coef[i]*imu_reflex[i])/( self.stance_coef+1)  # 归一化到1
        
        #print("coef swing",swing_coef_apply)
            
        
        
        
        # read feedback
        position_Read=get_position(self.robot,self._physics_client_id)
        self.flat_cpg_tick=self.theta_real_n[self.cpg_index] #6,3

        
    
        ## servo client 
        if self.on_reflex.any() or imu_reflex.any():
            set_position_multi(theta_new_sim,self.robot_servo,self.servo_client)
            position_servo=get_position(self.robot_servo,self.servo_client)
        for i in range(6):
            if self.on_reflex[i]==1:
                self.flat_cpg_tick[i]=position_servo[i]
            elif imu_reflex[i]==1:
                self.flat_cpg_tick[i]=position_servo[i]

         
            ## buffer 升级
        for i_index in range(3):
            self.traj_error_buf[i_index]=self.traj_error_buf[i_index+1]
        self.traj_error_buf[3]=(self.flat_cpg_tick-position_Read).flatten()/3.1415926*2048

        
        # 更新计数器
        if self.step_count==479:
            self.step_count=0
        else:
            self.step_count=self.step_count+1
        if self.step_count%self.T==int(self.T/4-1):
            self.T_count+=1
        
        
        self.dt_rewards=self.dt*20
        
        

        state=self.__get_observation()
        contact_force=np.zeros(12)
        
        torque_error_all,torque_target_all=self.__get__toruqe()

        #contact_force=[state[index,6] for index in range(6) if state[index,6]*self.phase_r[self.cpg_index,index] >0]
        #contact_force=[state[index,6] for index in range(6) if state[index,6] >0]
        basePos, baseOri = p.getBasePositionAndOrientation(self.robot, physicsClientId=self._physics_client_id)
        (x, y, z, w) = baseOri
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        IMU = np.array([roll, pitch, yaw])


        basePos, baseOri = p.getBasePositionAndOrientation(self.robot, physicsClientId=self._physics_client_id)
        (new_x, new_y, new_z) = basePos
        # print(new_z)
        joint_torques = self.__get_joints_torque()

        # 奖励
        if self.count<self.buffer_num:
            self.reward_buff[self.count]= (new_y - old_y) / self.dt_rewards 
            r_l_sum=0
            for index_i in range(self.count):
                r_l_sum+=self.reward_buff[index_i]
            r_linear_forward=r_l_sum/(self.count+1)

        else:
            for index_i in range(self.buffer_num-1):
                self.reward_buff[index_i]=self.reward_buff[index_i+1]
            self.reward_buff[self.buffer_num-1]=(new_y - old_y) / self.dt_rewards
            r_linear_forward=sum(self.reward_buff)/self.buffer_num
        
        
        #r_linear_forward = pow((new_y - old_y) / self.self.dt_rewards, 2) 
        # r_linear_forward=pow(new_y,2)
        # 　惩罚侧向的偏移
        
        if self.count<self.buffer_num_x:
            self.reward_buff_x[self.count]= -abs((new_x - old_x) / self.dt_rewards)- abs(new_x) * self.dt_rewards * 20 
            r_l_sum=0
            for index_i in range(self.count):
                r_l_sum+=self.reward_buff_x[index_i]
            r_lateral_move=r_l_sum/(self.count+1)

        else:
            for index_i in range(self.buffer_num_x-1):
                self.reward_buff_x[index_i]=self.reward_buff_x[index_i+1]
            self.reward_buff_x[self.buffer_num_x-1]=-abs((new_x - old_x) / self.dt_rewards)- abs(new_x) * self.dt_rewards * 20
            r_lateral_move=sum(self.reward_buff_x)/self.buffer_num_x
        
        #r_lateral_move = -abs((new_x - old_x) / self.dt_rewards) - abs(new_x) * self.dt_rewards * 0
        # 惩罚不稳定性
        r_instablity = -0.6 * abs(sum(IMU)) - 1 * abs(IMU[0])
        #r_instablity=0
        # 惩罚机器人与障碍物的碰撞，以及没有及时抬脚,惩罚摇摆相位出现力反馈
        if self.count<self.buffer_num_c:
            self.reward_buff_c[self.count]= -np.linalg.norm(contact_force)
            r_l_sum=0
            for index_i in range(self.count):
                r_l_sum+=self.reward_buff_c[index_i]
            r_collision=r_l_sum/(self.count+1)

        else:
            for index_i in range(self.buffer_num_c-1):
                self.reward_buff_c[index_i]=self.reward_buff_c[index_i+1]
            self.reward_buff_c[self.buffer_num_c-1]=-np.linalg.norm(contact_force)
            r_collision=sum(self.reward_buff_c)/self.buffer_num_c
        #r_collision = -np.linalg.norm(contact_force)
        
        #扭矩
        if self.count<self.buffer_num_t:
            self.reward_buff_t[self.count]= -np.linalg.norm(joint_torques)
            r_l_sum=0
            for index_i in range(self.count):
                r_l_sum+=self.reward_buff_t[index_i]
            r_torque=r_l_sum/(self.count+1)

        else:
            for index_i in range(self.buffer_num_t-1):
                self.reward_buff_t[index_i]=self.reward_buff_t[index_i+1]
            self.reward_buff_t[self.buffer_num_t-1]=-np.linalg.norm(joint_torques)
            r_torque=sum(self.reward_buff_t)/self.buffer_num_t    
        #r_torque = -np.linalg.norm(joint_torques)
        # print(f'linear_forward:{r_linear_forward}',f' r_lateral_move:{ r_lateral_move}')
        # print(f'r_instablity:{r_instablity}', f'r_collision:{r_collision}')
        reward1 = np.array([r_linear_forward, r_lateral_move, r_instablity, r_collision])
        w_linear_forward = 3000
        w_lateral_move = 80
        w_instablity = 30
        w_collision = 0.5
        w_torque = 1 
        
        
        # more smooth 
        #w_instablity = 25
        
        
        reward2 = np.array(
            [min(w_linear_forward * r_linear_forward,80), w_lateral_move * r_lateral_move, w_instablity * r_instablity,
             max(w_collision * r_collision, -20), max(w_torque * r_torque, -20)])
        reward = sum(reward2)
        if new_y>0.8:
            reward+=10 
        rewards=[[reward]]*self.agent_num #(1,6)
        if self.print_rewards:
            print("reward:                            ",reward,"rewards_n",reward2,)
        
        self.count+=1
        

        

        # 判断是否结束
        if self.current_T>self.tf:
            
            done=True
            #print("y:",new_y)
        
        else:
            done=False
        
                    
        #debug
        dones=[done]*self.agent_num
        share_obs=self.__get_share_observation()
        #info={"cpg_theta1":theta1,"action__theta":d_theta,"theta":theta,"state":state,"basePOs":basePos,"rewards2":reward2,"rewards1":reward1,"IMU":IMU,}
        info={"cpg_theta1":theta_new_sim,"action_coef":action_coef,"theta0":theta_sim,"state":state,"basePOs":basePos,"rewards2":reward2,"rewards1":reward1,"IMU":IMU,'height_map':self.height_map,"torque_target_all":torque_target_all,"torque_error_all":torque_error_all}
        available_actions=np.ones((6,1))
        #return state,rewards,dones,info
        return state,share_obs,rewards,dones,info,available_actions










    def reset(self):
        self.reward_buff=np.zeros(self.buffer_num)
        self.reward_buff_x=np.zeros(self.buffer_num_x)
        self.reward_buff_c=np.zeros(self.buffer_num_c)
        self.reward_buff_t=np.zeros(self.buffer_num_t)
        self.count=0
        self.current_T=0
        self.count_episode=self.count_episode+1
        
        self.cpg_index=self.ini_index_r[0]
    
        #self.random_seed=self.random_seed+1
        
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                     cameraPitch=-40, cameraTargetPosition=[0.2, -0.6, 1])
        
        if self.random_pos==1:
            self.seed_count+=1
            np.random.seed(self.seed_count+self.random_seed)
            ran_a0=np.random.randint(low=-20, high=20, size=2)*0.01
            print("seed",self.random_seed," robot x",ran_a0[0])
            
            self.startPos=[ran_a0[0],-0.05, 0.1]
        
        p.resetBasePositionAndOrientation(self.robot,self.startPos,self.startOri)
        #p.resetBasePositionAndOrientation(self.slope,self.Pos_slope,self.Ori)
        Pos = [0.1, 0, 0.05]
        for j in range(1):
            x_ran = random.uniform(-0.3, 0.3)
            y_ran = random.uniform(-0.05, 0.3)
            Pos = [x_ran, y_ran, 0.05]
            #p.resetBasePositionAndOrientation(self.box[j],Pos,self.startOri)
            
        
        if self.load_steps and  self.change_steps:
            height=0.2 
            length=0.3
            
            step_height_ran = random.uniform(0.00, 0.03)
            #self.step_height=step_height_ran
            print("seed",self.random_seed," robot x",ran_a0[0])
            
            np.random.seed(self.random_seed)
            x_range=np.arange(-self.step_width*(2*10-1),self.step_width*(2*10+1),self.step_width*2) 
            y_range=np.arange(self.step_width*(2*2-1),self.step_width*(2*22-1),self.step_width*2)
            self.step_start_place=[x_range[0]-self.step_width,y_range[0]-self.step_width]
            self.height_map=np.zeros((20,20)) 
            for i in range(20):
                for j in range(20):
                    if j<10:
                        hei_b=np.random.random()*self.step_height*(2)-height
                    else:
                        hei_b=np.random.random()*self.step_height*(2)-height+self.step_height
                    base_position=[x_range[i],y_range[j],hei_b] 
                    p.resetBasePositionAndOrientation(self.box[20*i+j],base_position,self.startOri)
                    self.height_map[i,j]=hei_b +height  
        
        
        if self.load_blocks :
            
            x_ran_a0=np.random.randint(low=-10, high=10, size=10)*0.03
            #x_ran_a=np.unique(x_ran_a0)[:10]
            np.random.shuffle(x_ran_a0)
            y_ran_a0=np.random.randint(low=5, high=25, size=10)*0.01
            y_ran_a1=np.random.randint(low=25, high=45, size=10)*0.01
            #y_ran_a=np.unique(y_ran_a0)[:10]
            x_a=np.arange(-15,15,2)*0.03
            np.random.shuffle(x_a)
            y_a=np.arange(-0,50,3)*0.01
            x_1=[-0.25,-0.1,0.05,0.2,0.35,-0.35,-0.2,-0.05,0.1,0.25]
            y_1=np.arange(10,30,2)*0.01  
            y_2=np.arange(30,50,2)*0.01  
            
            
            for j in range(6):
                
                Pos = [x_ran_a0[j], y_ran_a0[j], self.h_box] 
                if self.eval_random:
                    p.resetBasePositionAndOrientation(self.box[j],Pos,self.startOri)
            for j in range(10):
                if self.eval_random:
                    Pos = [x_ran_a0[j], y_ran_a1[j], self.h_box] 
                    p.resetBasePositionAndOrientation(self.box[j+10],Pos,self.startOri)    
           
        
        
        '''   
        p.resetBasePositionAndOrientation(self.Box1,self.Pos1,self.startOri)
        p.resetBasePositionAndOrientation(self.Box2,self.Pos2,self.startOri)
        p.resetBasePositionAndOrientation(self.Box3,self.Pos3,self.startOri)
        '''
        for j in range(18):
            p.changeDynamics(self.robot,j,lateralFriction=0.7)
            dyn = p.getDynamicsInfo(self.robot,j)
            f_friction_lateral=dyn[1]
            #print(f_friction_lateral,j)
        #print("box")
        for j in range(len(self.box)):
            p.changeDynamics(self.box[j], -1, lateralFriction=0.8)
            dyn = p.getDynamicsInfo(self.box[j],-1)
            f_friction_lateral=dyn[1]
        self.__set_position(self.theta_ini_r)
        for j in range(120):
            p.stepSimulation(physicsClientId=self._physics_client_id)
            
        phase_now=self.phase_r[self.cpg_index]   
        for i in range(6):
            if phase_now[i]==1:#摇摆相
                self.coef_real[i]=(1)/(self.swing_coef+1)
            elif phase_now[i]==-1:#支撑相
                self.coef_real[i]=(1)/( self.stance_coef+1)  # 归一化到1
        obs=self.__get_observation()
        share_obs=self.__get_share_observation()
        available_actions=np.ones((6,2))
        
        
        
        ## 相关参数初始化
        # reflex 相关参数的初始化
        self.traj_error_buf=np.zeros((4,18))
        self.on_reflex=np.zeros(6)
        self.reflex_index=np.ones(6)*(-10)
        self.on_reflex_stance=np.zeros(6)
        self.reflex_index_stance=np.ones(6)*(-10)
        self.stance_step_per_reflex=np.ones(6)*(0)
        self.swing_step_per_reflex=np.ones(6)*(0)
        self.sum_leg_all=[]
        self.reflex_real_all=[]
        self.on_reflex_all=[]
        self.traj_error_all=[]
        self.count_all=[]
        self.csv_rows=[]
        self.coef_real=np.zeros(6)
        self.last_reflex_coef=np.ones(6)
        
        
        
        self.T=240
        self.T_count=0
        coef=2
        self.coef_stance=1
        self.reflex=np.zeros(6)
        # 计数在每个swing or reflex中的步数
        self.swing_step_count=0
        self.reflex_count=np.zeros(6)
        self.step_count=0
        
        
        
        
        for j in range(18):
            p.changeDynamics(self.robot,j,lateralFriction=1,physicsClientId=self._physics_client_id)
            dyn = p.getDynamicsInfo(self.robot,j,physicsClientId=self._physics_client_id)
            f_friction_lateral=dyn[1]
            #print(f_friction_lateral,j)
        p.changeDynamics(self.plane, -1, lateralFriction=3,physicsClientId=self._physics_client_id)
        for j in range(1):
            p.changeDynamics(self.box[j], -1, lateralFriction=3,physicsClientId=self._physics_client_id)
            dyn = p.getDynamicsInfo(self.box[j],-1)
            f_friction_lateral=dyn[1]
            #print(f_friction_lateral,j)
            
        # servo client robot 初始化为init
        for i in range(40):
            set_position_multi(self.theta_ini_r, self.robot_servo, self.servo_client)
        
        

        return obs,share_obs,available_actions


    def render(self, mode='human', close=False):
        p.setGravity(0, 0, -9.8, physicsClientId=self._physics_client_id)

        location, _ = p.getBasePositionAndOrientation(self.robot)
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0,
                                     cameraPitch=-40, cameraTargetPosition=location)
        
    def seed(self,seed0):
        self.random_seed=seed0

    def close(self):
        if self._physics_client_id >= 0:
            p.disconnect()
        self._physics_client_id = -1
    
    def set_random_seed(self,seed):
        self.random_seed=seed
    
    

