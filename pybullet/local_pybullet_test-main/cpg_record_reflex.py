import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data as pd
import time
import math
from CPGs import *
import random
import json
from pybullet_functions import *
from reflex_related import *


# 插值




    


def __get_reward(old_base_po,count):
    (old_x,old_y,old_z)=old_base_po
    state = __get_observation(robot,box,cpgs)
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
    joint_torques = __get_joints_torque(robot)
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
    observation0=__get_observation1(data_read,cpg_index)
    end_pos=observation0[:,8:11]
    loss_end_pos=-np.linalg.norm(end_pos)
    w_end_pos=5000
    
    w_linear_forward = 500
    w_lateral_move = 100
    w_instablity = 8
    w_collision = 0.5
    w_torque = 0.5  
    reward2 = np.array([w_linear_forward * r_linear_forward, w_lateral_move * r_lateral_move, w_instablity * r_instablity,
                        max(w_collision * r_collision, -20), max(w_torque * r_torque, -20),loss_end_pos*w_end_pos])
    
    reward_all=sum(reward2)
    if new_y>0.8:
        print("y",new_y)
        reward_all+=10
    else:
        if new_y<0.5:
            print("y",new_y)
            reward_all-=20

    #print(reward_all)
    return reward2



def memory(count,pos_error):
    lambda0=0.6
    r_l_sum=0
    if count<buffer_num_error:
        accumulate_error_buffer[count]= pos_error 
        
        for index_i in range(count):
            r_l_sum+=pow(lambda0,count-index_i)*accumulate_error_buffer[index_i]
        #r_lateral_move=r_l_sum/(count+1)

    else:
        for index_i in range(buffer_num_error-1):
            accumulate_error_buffer[index_i]=accumulate_error_buffer[index_i+1]
            r_l_sum+=pow(lambda0,count-index_i)*accumulate_error_buffer[index_i]

        accumulate_error_buffer[buffer_num_error-1]=pos_error
        r_l_sum+=pos_error
        #r_lateral_move=sum(accumulate_error_buffer)/buffer_num_error
    return r_l_sum

main_client=p.connect(p.GUI)  # 连接到仿真服务器
#main_client=p.connect(p.DIRECT)  # 连接到仿真服务器
p.setGravity(0, 0, -9.8,physicsClientId=main_client)  # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath(),physicsClientId=main_client)  # 设置pybullet_data的文件路径

# 加载地面
floor = p.loadURDF("plane.urdf",physicsClientId=main_client)

# mini cheetah的初始位置
startPos = [0, 0, 0.1]
box = []
# 加载urdf文件
robot = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos,physicsClientId=main_client)



## client2
servo_client=p.connect(p.DIRECT)
#servo_client=p.connect(p.GUI)
p.setGravity(0, 0, -9.8,physicsClientId=servo_client)  # 设置重力值
p.setAdditionalSearchPath(pd.getDataPath(),physicsClientId=servo_client)  # 设置pybullet_data的文件路径

# 加载地面
floor_servo = p.loadURDF("plane.urdf",physicsClientId=servo_client)
box_servo=[]
box_servo.append(floor_servo)

startPos_servo=[0,0,1]
# 加载urdf文件
robot_servo = p.loadURDF("hexapod_34/urdf/hexapod_34.urdf", startPos_servo,physicsClientId=servo_client,useFixedBase=True,)

# rewards buffer
buffer_num=240
buffer_num_x=90
buffer_num_c=90
buffer_num_t=90
buffer_num_error=1
reward_buff=np.zeros(buffer_num)
reward_buff_x=np.zeros(buffer_num_x)
reward_buff_c=np.zeros(buffer_num_c)
reward_buff_t=np.zeros(buffer_num_t)
accumulate_error_buffer=np.zeros((buffer_num_error,6,3))
for j in range(30):
    x_ran = random.uniform(-0.4, 0.4)
    y_ran = random.uniform(0.0, 0.5)
    Pos = [x_ran, y_ran, 0.0125]
    box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos,useFixedBase=True,physicsClientId=main_client))

height=0.02
length=5
width=0.03
base_position=[0,0,0.01]
hal_geo=[length, width, height]
colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=hal_geo,physicsClientId=main_client)
visualID=p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=hal_geo,physicsClientId=main_client) 
ori=  rpy2quaternion(0,0,0)                             
#box1=p.createMultiBody(0, colBoxId,visualID,base_position,ori)

for i in range(10):
  base_position=[0,-0.484+0.4*(i+2),0.025]
  #box.append(p.createMultiBody(0, colBoxId,visualID,base_position,ori))







Pos = [0.25, 0.4, 0.2]
# box1=p.loadURDF("box_80_60_30/box2.urdf",Pos)
#box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos))
Pos2 = [0.4, 0.7, 0.2]
# box2=p.loadURDF("box_80_60_30/box2.urdf",Pos2)
#box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos2))
Pos3 = [-3.5, 0.8, 0.2]
# box3=p.loadURDF("box_80_60_30/box2.urdf",Pos3)
#box.append(p.loadURDF("box_80_60_30/box2.urdf", Pos3))
#box3=p.loadURDF("box_40_30_20/box1.urdf",Pos3)

box.append(floor)

Pos_slope=[-1,-0.3,0]
Ori=[0.0, 0.0, 0.7071067811865475, 0.7071067811865476]
#slope=p.loadURDF("slope_20/urdf/slope_20.urdf", Pos_slope,Ori,useFixedBase=True)
#box.append(slope)


for j in range(18):
    p.changeDynamics(robot,j,lateralFriction=1,physicsClientId=main_client)
    dyn = p.getDynamicsInfo(robot,j,physicsClientId=main_client)
    f_friction_lateral=dyn[1]
    print(f_friction_lateral,j)
print("box")
p.changeDynamics(floor, -1, lateralFriction=3,physicsClientId=main_client)
for j in range(1):
    p.changeDynamics(box[j], -1, lateralFriction=3,physicsClientId=main_client)
    dyn = p.getDynamicsInfo(box[j],-1,physicsClientId=main_client)
    f_friction_lateral=dyn[1]
    print(f_friction_lateral,j)
# 获取关节数量
numJoints = p.getNumJoints(robot,physicsClientId=main_client)
# 设置机器人的视觉效果，这里参数-1指的是机器人的base link。
# 我们可以通过rgba值改变机器人相关link的颜色
p.changeVisualShape(robot, -1, rgbaColor=[1, 1, 1, 1],physicsClientId=main_client)

cf_base = 0.35  # 右边的零位状态　左边增加负号
ft_base = -0.35

pos = [0, -cf_base, -ft_base, 0, -cf_base, -ft_base, 0, -cf_base, -ft_base,
       0, cf_base, ft_base, 0, cf_base, ft_base, 0, cf_base, ft_base]

p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0,
                             cameraPitch=-70, cameraTargetPosition=[0.2, -0.6, 1],physicsClientId=main_client)
# 设置每个link的颜色，并且设置初始关节角度
for j in range(numJoints):
    force = 30
    p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, pos[j],force=4,physicsClientId=main_client)

# 　调整至零位
dt = 1. / 1000.
p.setTimeStep(dt,physicsClientId=main_client)

p.stepSimulation(physicsClientId=main_client)

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
## 之前相对可行的是13
with open('force_real17.json', 'r') as f:
    data_read = json.load(f)
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    theta_ini_r=np.asarray(data_read['theta_ini'])
    ini_index_r=np.asarray(data_read['ini_index'])
    end_pos=np.asarray(data_read['end_pos'])
    theta_real_n=np.asarray(data_read['theta_real_n'])

set_position(theta_ini_r,robot)
p.stepSimulation(physicsClientId=main_client)
for j in range(240 * 1):
    p.stepSimulation(physicsClientId=main_client)

tf = 10
start_T = time()
#zz_n = cpgs.two_layer_out_all(zz0, 0, tf)
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
cpg_index=ini_index_r
contacts = np.zeros((1, 6))
T = [0]
theta1_n = np.zeros((1, 6, 3))
phase1_n = np.zeros((1, 6))
reward_n=np.zeros((1, 6))
reward_n_all=[0]
end_pos_n=np.zeros((1,6,3))
#plt.ion()
#plt.figure(1)
#plt.Figure()
cpgs.old_zz=cpg_r[:,:,cpg_index]
force_diff_n=np.zeros((1, 6))
phase_n=np.zeros((1, 6))
last_error=np.zeros((6,3))






cpg_index=ini_index_r[0]
set_position_multi(theta_ini_r,robot,main_client)


reflex=1
if reflex:
# init vaiables
    positions=[]
    traj_error_buf=np.zeros((4,18))
    on_reflex=np.zeros(6)
    reflex_index=np.ones(6)*(-10)
    on_reflex_stance=np.zeros(6)
    reflex_index_stance=np.ones(6)*(-10)
    stance_step_per_reflex=np.ones(6)*(0)
    swing_step_per_reflex=np.ones(6)*(0)
    sum_leg_all=[]
    reflex_real_all=[]
    on_reflex_all=[]
    traj_error_all=[]
    count_all=[]
    csv_rows=[]
    
    
    step=0
    T=240
    T_count=0
    coef=2
    coef_stance=1
    step=0
    reflex=np.zeros(6)
    # 计数在每个swing or reflex中的步数
    swing_step_count=0
    reflex_count=np.zeros(6)



    
    for count in range(int(T*12)):
        start_time_t=time()
        
        # 接收imu的数据
        IMU_data=get_IMU(robot)       
        #IMU_data=np.array([0,0,0,0,0,0,])
        (roll ,pitch,yaw)=IMU_data[0:3]/3.1415926*180 # 绕着 xy z轴转动
        print("roll pitch yaw: ",roll ,pitch,yaw)

        if cpg_index<239:
            cpg_index+=1
        else:
            cpg_index=0

            
        
        if count<4:
            # just give command
            theta_sim=theta_r[cpg_index]    
            set_position_multi(theta_sim,robot,main_client)
            
            # read feedback
            observation0= get_observation1(data_read,cpg_index,robot,cpgs,box,main_client)
            position_Read=observation0[0:6,0:3]
            
            flat_cpg_tick=theta_real_n[cpg_index] #6,3
            traj_error_buf[count]=(flat_cpg_tick-position_Read).flatten()/3.1415926*2048
            on_reflex=np.zeros(6)
            
        else:
            # judhe whwther or not is a reflex
            # 在判断一开始的reflex 之前就清零
            # judge reflex 
            if count%120==0:
                k=1
            phase_now=phase_r[cpg_index,:]
            last_phase=phase_r[cpg_index-1]
            contact_v,contact_l=get_contactPoints(p,robot,box)
            #print("contact lateral",contact_l)
            #reflex=judge_reflex_force(contact_l)
            reflex,sum_leg=judge_reflex(traj_error_buf)
            
            reflex_sim=swing_reflex(reflex,phase_now)
            # 判断是不是reflex 基础上的 reflex
            reflex_sim_new=judge_new_reflex(reflex_sim,swing_step_per_reflex)

            reflex_stance,sum_leg_stance=judge_reflex_stance(traj_error_buf)
            #reflex_stance_sim0=real_reflex_leg_to_sim(reflex_stance)
            reflex_stance_sim=stance_reflex(reflex_stance,phase_now)
            print("sum_leg",sum_leg,'\n')
            
            #print("count:",count,"reflex",reflex,"on reflex",on_reflex)
            # on_reflex 在sim的基础上
            
            #on_reflex=swing_reflex(on_reflex,phase_now)
            #on_reflex_stance=stance_reflex(on_reflex_stance,phase_now)
            if reflex_sim.any():
                reflex_index[reflex_sim>0]=T_count
                on_reflex[reflex_sim>0]=1

            # 每有一个新的reflex  swing_step_per_reflex   置1
            #    
            if reflex_sim_new.any():
                swing_step_per_reflex[reflex_sim_new>0]=0
                #reflex_count[reflex_sim_new>0]+=1

            if reflex_stance_sim.any():
                reflex_index_stance[reflex_stance_sim>0]=T_count
                on_reflex_stance[reflex_stance_sim>0]=1
                on_reflex_stance[on_reflex>0]=0
            if on_reflex_stance.any():
                stance_step_per_reflex[on_reflex_stance>0]+=1
            if on_reflex.any():
                swing_step_per_reflex[on_reflex>0]+=1
                
            swing_step_per_reflex[swing_step_per_reflex>239]=0
            stance_step_per_reflex[stance_step_per_reflex>239]=0
            
            
            # swing 和stance  交界处 error 清零
            swing_step_per_reflex[(last_phase*phase_now)<0]=0
            stance_step_per_reflex[(last_phase*phase_now)<0]=0
            swing_step_count+=1
            if (last_phase*phase_now)[0]<1:
                traj_error_buf=np.zeros_like(traj_error_buf)
                swing_step_count=0
                on_reflex=np.zeros_like(on_reflex)
                reflex_count=np.zeros_like(on_reflex)
                
            for i in range(6):# 当进入下一周期后 默认不在reflex范围内
                if (T_count-reflex_index[i])==1 and on_reflex[i]>0:
                    on_reflex[i]=0
                    reflex_index[i]=-2
                    swing_step_per_reflex[i]=0
                if (T_count-reflex_index_stance[i])==1 and on_reflex_stance[i]>0:
                    on_reflex_stance[i]=0
                    reflex_index_stance[i]=-2
                    stance_step_per_reflex[i]=0
            # 将reflex  都转换到仿真的维度中  使得能够与phase 相关
            #on_reflex_sim=real_reflex_leg_to_sim(on_reflex)
            #on_reflex_stance_sim=real_reflex_leg_to_sim(on_reflex_stance)     
            
            #theta_new_sim=get_reflex_theta(theta_sim,coef,reflex_sim,on_reflex_sim,phase_now,)
            
            
            
            
            
            if (swing_step_per_reflex==1).any() :
                reflex_count[swing_step_per_reflex==1]+=1
                for index_i in range(30):
                    set_position_multi(position_Read,robot_servo,servo_client)
                position_servo=get_position(robot_servo,servo_client)



            # imu reflex
            
            imu_reflex=judge_imu_reflex2(IMU_data,phase_now)
            coef_imu=get_imu_coef(IMU_data,imu_reflex)
            # coef imu  正数是往下压 负数是往上收   
            
            
            
            coef_delta=0.4
            if swing_step_count<10:
                coef=1*np.ones(6)
                #coef_stance=1
            else:
                coef=np.clip(1*np.ones(6)+coef_delta*(reflex_count),0,3)
                #coef_stance=1.5
                print("coef",coef)
                
            
            theta_sim=theta_r[cpg_index]    
            
            theta_new_sim0=get_reflex_theta_all(theta_sim,coef,coef_stance,reflex_sim,on_reflex,reflex_stance_sim,on_reflex_stance,phase_now,stance_step_per_reflex,swing_step_per_reflex)
            # imu reflex
            theta_new_sim=imu_roll_reflex2(imu_reflex,phase_now,theta_new_sim0,coef_imu)
            
            
            
            set_position_multi(theta_new_sim,robot,main_client)

            
            
            # read feedback
            observation0= get_observation1(data_read,cpg_index,robot,cpgs,box,main_client)
            position_Read=observation0[0:6,0:3]
            flat_cpg_tick=theta_real_n[cpg_index] #6,3



            ## servo client 
            if on_reflex.any():
                set_position_multi(theta_new_sim,robot_servo,servo_client)
                position_servo=get_position(robot,servo_client)
                

            
            
            
            
            for i in range(6):
                if on_reflex[i]==1:
                    flat_cpg_tick[i]=position_servo[i]
                elif imu_reflex[i]==1:
                    flat_cpg_tick[i]=position_servo[i]

         
            ## buffer 升级
            for i_index in range(3):
                traj_error_buf[i_index]=traj_error_buf[i_index+1]
            traj_error_buf[3]=(flat_cpg_tick-position_Read).flatten()/3.1415926*2048
            
            # csv writer
            #traj_error_all.append(flat_cpg_tick-position_Read_tick)
            #count_all.append(count)
            #reflex_real_all.append(reflex)
            #on_reflex_all.append(on_reflex)
            #sum_leg_all.append(sum_leg)
            on_reflex2=copy.copy(on_reflex)
            on_reflex_stance2=copy.copy(on_reflex_stance)
        
            reflex_index2=copy.copy(reflex_index)
            reflex_index_stance2=copy.copy(reflex_index_stance)
            
            sum_leg=[0,0,0,0,0,0]
            
            csv_row=[]
            csv_row=[count,T_count,sum_leg,reflex_sim,reflex_stance_sim,IMU_data/3.1415926*180,(flat_cpg_tick-position_Read)/3.14*2048]
            #csv_row=[count,T_count,sum_leg,reflex_sim,reflex_stance_sim,on_reflex2,on_reflex_stance2,reflex_index2,reflex_index_stance2,swing_step_count,flat_cpg_tick,position_Read,IMU_data,(flat_cpg_tick-position_Read)/3.14*2048]
            csv_rows.append(csv_row)
            
            
            
            
            
            
            
            
            
            
        
            #print("current",current_read,"\n")
            
            
            while (time()-start_time_t)*1000<20.00:
                1
            end_time_t=time()
            print("count:",count,"reflex",reflex_sim,"on reflex",on_reflex,'reflex_index',reflex_index,'T_count',T_count,"\n")
            print("current error",(flat_cpg_tick-position_Read)/3.14*2048)
           
            
        
        
        
        
            
        
        
        if step==479:
            step=0
        else:
            step=step+1
        if step%T==int(T/4-1):
            T_count+=1
        









with open('data_reflex_1.csv', mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['count','T_count','sum_leg','reflex_sim','reflex_stance_sim','on_reflex2','on_reflex_stance2','reflex_index2','reflex_index_stance2','swing_step_count','flat_cpg_tick','position_Read_tick','IMU_data'])
        writer.writerows(csv_rows)




basePos, baseOri = p.getBasePositionAndOrientation(robot,physicsClientId=main_client)
    
(new_x, new_y, new_z) = basePos

print("x y z",basePos)


plt.Figure()
for j in range(6):
    plt.subplot(3, 2, j+1)
    plt.plot(T[161:161+240*4], end_pos_n[161:161+240*4, j,1],'-r',)
    


plt.show()

plt.Figure()
for j in range(6):
    plt.subplot(3, 2, j+1)
    plt.plot(T[161:161+240*4], end_pos_n[161:161+240*4, j,0],'-r',)
    


plt.show()

plt.Figure()
for j in range(6):
    plt.subplot(3, 2, j+1)
    plt.plot(T[161:161+240*4], end_pos_n[161:161+240*4, j,2],'-r',)
    


plt.show()




plt.Figure()
for j in range(6):
    plt.subplot(3, 2, j+1)
    plt.plot(T[161:161+240*12], contacts[161:161+240*12, j],'-r',T[161:161+240*12],phase1_n[161:161+240*12,j],'-b')
    


plt.show()

class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)



theta_ini=theta_r[abs(theta_r[:,0,0])<1e-3][0]
ini_index=np.argwhere(abs(theta_r[:,0,0])<1e-3)[0]
contact1=(contacts[161:401,:]+contacts[161+240*1:401+240*1,:]+contacts[161+240*2:401+240*2,:]+contacts[161+240*3:401+240*3,:])/4
data = {'contact_force': contact1,'phase':phase1_n[161:401,:],'cpg':cpg_r,'theta':theta_r,'theta_ini':theta_ini,'ini_index':ini_index}
data_json = json.dumps(data, cls=NumpyArrayEncoder)
#print(f'contact force:{data_json}')

# 写入json文件

plt.Figure()
for j in range(6):
    plt.subplot(3, 3, j + 1)
    plt.plot(T, reward_n[:, j])


plt.subplot(3, 3, 7)
plt.plot(T, reward_n_all)
plt.show()