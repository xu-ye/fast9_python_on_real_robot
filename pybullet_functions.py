import math
import numpy as np
import pybullet as p



def get_IMU(robot):
    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    (x, y, z, w) = baseOri
    # print(basePos)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    IMU = np.array([roll, pitch, yaw])
    return IMU


def interp(theta_0, theta_1, n):
    theta_n = np.zeros((n, 6, 3))
    # theta_n=theta_0+(theta_1-theta_0)/n
    for j in range(n):
        theta_n[j, :, :] = theta_0 + (theta_1 - theta_0) / n * (j + 1)

    return theta_n


def setbc_position(theta_b,robot):
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


def set_position(theta,robot):
    for j in range(6):
        p.setJointMotorControl2(robot, 3 * j, p.POSITION_CONTROL, theta[j, 0])
        p.setJointMotorControl2(robot, 3 * j + 1, p.POSITION_CONTROL, theta[j, 1])
        p.setJointMotorControl2(robot, 3 * j + 2, p.POSITION_CONTROL, theta[j, 2])


def get_contactPoints(p,robot,box):
    result = np.zeros((6))
    contact_norm=np.zeros((6,3))
    contact_force_lateral=np.zeros((6))
    #phase = cpgs.get_phase(cpgs.old_zz)
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
                    contact_normal = f_contact[7]# 接触方向
                    contact_force_v = f_contact[9] * contact_normal[2] / np.linalg.norm(contact_normal)
                    contact_force_l = f_contact[9] * np.linalg.norm(contact_normal[0:1]) / np.linalg.norm(
                        contact_normal)
                    # print(contact_normal)
                    if link_index >= 0:
                        link_name = (p.getJointInfo(robot, link_index)[12]).decode()
                    else:
                        link_name = 'base'

        # result.append((link_name, f_contact[9]))
        result[j] =  1 *contact_force_v
        contact_force_lateral[j]=contact_force_l
    return result,contact_force_lateral




def get_observation(robot,box,cpgs):
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


def get_position(robot,client):
    robot_joints_states = p.getJointStates(robot, jointIndices=range(18),physicsClientId=client)
    robot_joint_positions = np.array([x[0] for x in robot_joints_states])
    positions=(np.array(robot_joint_positions)).reshape((6,3))
    return positions



def get_observation1(data_read,cpg_index,robot,cpgs,box,client):
    agent_num=6
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    #torque_n=np.asarray(data_read['torque_n'])
# IMU的数据
    basePos, baseOri = p.getBasePositionAndOrientation(robot)
    expeted_end_pos_r=np.asarray(data_read['end_pos'])
    
    

    observation_temp=[]
    phase = cpgs.get_phase(cpgs.old_zz)
    theta_old = cpgs.cpg2theta_reshape(cpgs.old_zz)

    for agent_index in range(agent_num):
        JointIndicies_agenti=[3*agent_index,3*agent_index+1,3*agent_index+2]
        robot_joints_states_agenti = p.getJointStates(robot, jointIndices=JointIndicies_agenti,physicsClientId=client)
        robot_joint_positions_agenti = np.array([x[0] for x in robot_joints_states_agenti])
        robot_joint_Torque_agenti = np.array([x[3] for x in robot_joints_states_agenti])
        world_end_position=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True,physicsClientId=client)[0]
        world_end_ori=p.getLinkState(robot,3*agent_index+2,computeForwardKinematics=True,physicsClientId=client)[0]
        
        #print(link_name)
        # 误差的error
        theta_error=robot_joint_positions_agenti-theta_old[agent_index,:]
        theta_target=theta_r[cpg_index,agent_index,:][0]
        theta_diff=theta_target-robot_joint_positions_agenti
        #print("theta_diff",theta_diff)
        #print("joint torque",robot_joint_Torque_agenti)
        #torque_target=torque_n[cpg_index,agent_index,:][0]
        #torque_error=torque_target-robot_joint_Torque_agenti
        #print("torque_error",torque_error)



        # 相位信息
        phase_continus1=cpgs.old_zz[agent_index,2]
        phase_continus2=cpgs.old_zz[agent_index,3]
        phase_continus=cpgs.old_zz[agent_index,2:4]
        #force_sin=force_r[abs(cpg_r[agent_index,2,:]-phase_continus1)<1e-3,agent_index]
        #force_cos=force_r[abs(cpg_r[agent_index,3,:]-phase_continus2)<1e-3,agent_index]
        #force_t=np.intersect1d(force_cos,force_sin)[0]
        force_t=force_r[cpg_index,agent_index]

        if agent_index>=3:
            end_pos,end_ori=get_forward_pos(-robot_joint_positions_agenti)
        else:
            end_pos,end_ori=get_forward_pos(robot_joint_positions_agenti)
        
        expeted_end_pos=expeted_end_pos_r[cpg_index,agent_index,:]
        end_pos_error=expeted_end_pos-end_pos
        #print("end pos error",end_pos_error*1000)
        #print("expected end pos",expeted_end_pos*1000)
        #print("end pos",end_pos*1000)



        

        # 竖直方向的接触力
        foot_contacts = []
        contact_force_v = 0
        contact_force_l = 0
        for body_1 in box:
            foot_contact = p.getContactPoints(bodyA=robot, bodyB=body_1, linkIndexA=3 * agent_index + 2,physicsClientId=client)
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
                    contact_force_l += f_contact[9] * np.linalg.norm(contact_normal[0:1]) / np.linalg.norm(contact_normal)
        contact_force_diff=contact_force_v-force_t
        #print("contact_force_diff ",contact_force_diff)


        observation_agenti = np.append(robot_joint_positions_agenti, robot_joint_Torque_agenti)
        observation_agenti = np.append(observation_agenti,contact_force_v)
        observation_agenti = np.append(observation_agenti,contact_force_diff)
        observation_agenti = np.append(observation_agenti,end_pos_error)
        observation_agenti = np.append(observation_agenti,end_ori)
        observation_agenti = np.append(observation_agenti,phase_continus)
        observation_agenti = np.append(observation_agenti,theta_error)


        observation_temp.append(observation_agenti)

    observation = np.array(np.vstack(observation_temp),dtype=np.float64)
    return observation        

def get_forward_pos(pos):
    theta_1=0+pos[0]
    theta_2=-(90-21.36-0)/180*3.1415926+pos[1]
    theta_3=(180-(62.69+0-21.36-15))/180*3.1415926-pos[2]
    
    T_01=np.array([[np.cos(theta_1),-np.sin(theta_1),0,0],[np.sin(theta_1),np.cos(theta_1),0 ,0],[0, 0 ,1, -33.33/1000],[0, 0, 0 ,1]])
    T_12=np.array([[np.cos(theta_2),-np.sin(theta_2),0,42.55/1000],[0,0,1,0],[-np.sin(theta_2),-np.cos(theta_2),0,0],[0,0,0,1]])
    T_23=np.array([[np.cos(theta_3),-np.sin(theta_3),0,64.25/1000],[np.sin(theta_3),np.cos(theta_3),0,0],[0,0,1,0],[0,0,0,1]])
    T_34=np.array([[1,0,0,135.4/1000],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    T_04=T_01@T_12@T_23@T_34
    end_pos=T_04[0:3,3]
    ouler_angles=rotationMatrixToEulerAngles(T_04[0:3,0:3])
    return end_pos,ouler_angles       

def rotationMatrixToEulerAngles(R) :

    
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

def __get_joints_torque(robot):
        joint_torque = np.zeros(18)
        for j in range(18):
            joint_torque[j] = p.getJointState(robot, j)[3]
        return joint_torque





def rpy2quaternion(roll, pitch, yaw):
    x=math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    y=math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)+math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    z=math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2)
    w=math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2)-math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2)
    ori=np.array([x,y,z,w])
    return ori


def set_position_multi(theta,robot,client):
        # theta (6,3) 单位弧度  1=在当前目标下运动10ms
        #0.01647,0.397,8.92
        p.setTimeStep(1/1000,physicsClientId=client)
        frictionForce=0.9
        P_gain=0.01647
        D_gain=0.397
        max_speed=8.92
        Force=4.1
        p.setJointMotorControlArray(robot, range(18), p.VELOCITY_CONTROL, targetVelocities=[0]*18, forces=[frictionForce]*18,physicsClientId=client)
                    # 直接应用real tau 完全没有按照既定轨迹运行
        theta1=theta.flatten()  
        for tick_num in range(20):         
            for actuator_index in range(18):
                p.setJointMotorControl2(bodyUniqueId=robot,jointIndex=actuator_index,controlMode=p.POSITION_CONTROL,targetPosition=theta1[actuator_index],
                positionGain=P_gain,
                velocityGain=D_gain,
                maxVelocity=max_speed,
                force = Force,physicsClientId=client
                )
            p.stepSimulation(physicsClientId=client)    

