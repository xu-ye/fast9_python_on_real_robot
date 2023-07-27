import json
from sys import path
path.append("../../")
import math
import numpy as np
import random
#import Servos


class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)




def sim_angles_to_real0(theta):
    # theta [6,3]
    real_angles=np.zeros_like(theta).flatten()
    for i in range(6):
        if i<3:
            
            hip=180+theta[i,0]/math.pi*180 
            
            knee=180-theta[i,1]/math.pi*180
            ankle=62.69+theta[i,2]/math.pi*180
            
            real_angles[3+i*6+0]=hip
            real_angles[3+i*6+1]=knee
            real_angles[3+i*6+2]=ankle
            
        else:
            hip=180+theta[i,0]/math.pi*180 
            knee=180+theta[i,1]/math.pi*180
            ankle=62.69-theta[i,2]/math.pi*180
            
            real_angles[0+(i-3)*6+0]=hip
            real_angles[0+(i-3)*6+1]=knee
            real_angles[0+(i-3)*6+2]=ankle
        
    return real_angles

def angles_to_tick(angles):
    theta_tick=np.zeros_like(angles)
    for i in range(18):
        theta_tick[i]=int(angles[i]/180*2048)
    return theta_tick
            

def real_angles_to_sim0(real_angles):
    # theta [6,3]
    theta=np.zeros((6,3))
    for i in range(6):
        if i<3:
            theta[i,0]= (real_angles[3+i*6+0]-180)/180*math.pi
            theta[i,1]= (-real_angles[3+i*6+1]+180)/180*math.pi
            theta[i,2]= (real_angles[3+i*6+2]-62.69)/180*math.pi
            
            
            
        else:
            theta[i,0]= (real_angles[0+(i-3)*6+0]-180)/180*math.pi
            theta[i,1]= (real_angles[0+(i-3)*6+1]-180)/180*math.pi
            theta[i,2]= (-real_angles[0+(i-3)*6+2]+62.69)/180*math.pi
            
           
        
    return theta      

def real_angles_to_sim(real_angles):
    # theta [6,3]
    
    theta=np.zeros((6,3))
    for i in range(6):
        if i<3:
            theta[i,0]= -(real_angles[0+i*6+0]-180)/180.0*math.pi
            theta[i,1]= (-real_angles[0+i*6+1]+180)/180.0*math.pi
            theta[i,2]= (real_angles[0+i*6+2]-62.69)/180.0*math.pi
            
            
            
        else:
            theta[i,0]= -(real_angles[3+(i-3)*6+0]-180)/180.0*math.pi
            theta[i,1]= (real_angles[3+(i-3)*6+1]-180)/180.0*math.pi
            theta[i,2]= (-real_angles[3+(i-3)*6+2]+62.69)/180.0*math.pi
            
           
        
    return theta

def sim_angles_to_real(theta):
    # theta [6,3]  输出 np array [18]
    real_angles=np.zeros_like(theta).flatten()*1.00000
    for i in range(6):
        if i<3:
            
            hip=(180-theta[i,0]/math.pi*180) 
            
            knee=180-theta[i,1]/math.pi*180
            ankle=62.69+theta[i,2]/math.pi*180
            
            real_angles[0+i*6+0]=hip
            real_angles[0+i*6+1]=knee
            real_angles[0+i*6+2]=ankle
            
        else:
            hip=(180-theta[i,0]/math.pi*180) 
            knee=180+theta[i,1]/math.pi*180
            ankle=62.69-theta[i,2]/math.pi*180
            
            real_angles[3+(i-3)*6+0]=hip
            real_angles[3+(i-3)*6+1]=knee
            real_angles[3+(i-3)*6+2]=ankle
        
    return real_angles    

def real_current_to_sim_torque(real_current):
    # theta [6,3]
    torque_sim=np.zeros((6,3))
    real_torque1=real_current.reshape(6,3)
    for i in range(6):
        if i<3:
            torque_sim[i,0]= real_current[3+i*6+0]*2.69/1000*1.82-0.2576
            torque_sim[i,1]= -(real_current[3+i*6+1]*2.69/1000*1.82-0.2576)
            torque_sim[i,2]= real_current[3+i*6+2]*2.69/1000*1.82-0.2576
            
            
            
        else:
            torque_sim[i,0]= real_current[0+(i-3)*6+0]*2.69/1000*1.82-0.2576
            torque_sim[i,1]= real_current[0+(i-3)*6+1]*2.69/1000*1.82-0.2576
            torque_sim[i,2]= -(real_current[0+(i-3)*6+2]*2.69/1000*1.82-0.2576)
            
           
        
    return torque_sim     

def real_torque_to_sim_torque(real_current):
    # theta [6,3]
    torque_sim=np.zeros((6,3))
    
    for i in range(6):
        if i<3:
            torque_sim[i,0]= real_current[3+i*6+0]
            torque_sim[i,1]= -(real_current[3+i*6+1])
            torque_sim[i,2]= real_current[3+i*6+2]
            
            
            
        else:
            torque_sim[i,0]= real_current[0+(i-3)*6+0]
            torque_sim[i,1]= real_current[0+(i-3)*6+1]
            torque_sim[i,2]= -(real_current[0+(i-3)*6+2])
            
           
        
    return torque_sim            

def generate_traj(data_read,fre):
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    theta_ini_r=np.asarray(data_read['theta_ini'])
    ini_index_r=np.asarray(data_read['ini_index'])
    torque_n=np.asarray(data_read['torque_n'])
    cpg_index=ini_index_r[0]
    theta_new=[]
    phase_new=[]
    inter_num=int(1/fre)
    
    for step in range(240):
        if cpg_index==239:
            cpg_next=0
        else:
            cpg_next=cpg_index+1
        for inter_index in range(inter_num):
            theta_temp=theta_r[cpg_index,:,:]+(theta_r[cpg_next,:,:]-theta_r[cpg_index,:,:])/inter_num*inter_index
            theta_new.append(theta_temp)
            phase_temp=phase_r[cpg_next] # -1 support00 1 swing
            phase_new.append(phase_temp)
        cpg_index=cpg_next
    return theta_new,phase_new
            
        
        
        
    

with open('force_real7.json', 'r') as f:
    
    data_read = json.load(f)
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    theta_ini_r=np.asarray(data_read['theta_ini'])
    ini_index_r=np.asarray(data_read['ini_index'])
    torque_n=np.asarray(data_read['torque_n'])

step= 0
cpg_index=ini_index_r[0]
fre=0.5
theta_new,phase_new=generate_traj(data_read,fre)


import numpy as np

from Servos import *


servos=Servos()
#servos.light_LED()
#goal_position=2048*np.ones(1,18)
servos.read_voltage(1
                    )
servos.set_position_control()
goal_position=np.array([180,204,85,180,204,85])
position_Read=servos.read_position_loop()
print("read position:",position_Read)
DXLn_ID=[0,1,2,3,4,5]
servos.enable_torque(DXLn_ID)
#print("Press any key to continue! (or press ESC to move leg2!)")
#if getch() == chr(0x1b):
#    servos.write_some_positions(goal_position,DXLn_ID)
time.sleep(1)
DXLn_ID=[6,7,8,9,10,11]
servos.enable_torque(DXLn_ID)
#print("Press any key to continue! (or press ESC to move leg2!)")
#if getch() == chr(0x1b):
#    servos.write_some_positions(goal_position,DXLn_ID)


DXLn_ID=[12,13,14,15,16,17]
servos.enable_torque(DXLn_ID)
#print("Press any key to continue! (or press ESC to move leg2!)")
#if getch() == chr(0x1b):
#    servos.write_some_positions(goal_position,DXLn_ID)

position_Read=servos.read_all_positions()
print("read position:",position_Read)
positions_tick=[]
phase_all=[]
goal_pos_sim=[]
current_pos_tick=[]
step=0
theta_sim=theta_new[step]

    
angles_real=sim_angles_to_real(theta_sim)

servos.Robot_initialize(angles_real)
time.sleep(10)


# -0.3 0.33
T=int(1/fre)*240
position_Read=servos.read_all_positions()
position_Read=servos.read_all_positions()
position_Read=servos.read_all_positions()
position_Read=servos.read_all_positions()
position_Read=servos.read_all_positions()

for index in range(T*1):
    
    
    
    


    start_time_t=time.time()
    #theta_sim=theta_r[cpg_index,:,:]
    theta_sim=theta_new[step]
    phase=phase_new[step]
    
    angles_real=sim_angles_to_real(theta_sim)
    theta_tick=angles_to_tick(angles_real)
    
    
    positions_tick.append(theta_tick)
    phase_all.append(phase)
    goal_pos_sim.append(theta_sim)
    
    
    servos.write_all_positions(theta_tick)
    #servos.write_all_positions_angles(angles_real)
    #position_goal = np.trunc(angles_real / 360 * 4096)  # tick
    
    
    position_Read=servos.read_all_positions()
    position_tick=angles_to_tick(position_Read)
    
    current_pos_tick.append(position_tick)
    
    #position_error=angles_real-position_Read
    #print("position_error",position_error)
    #print("position target",angles_real)
    
    
    #current_read=servos.read_all_torque()
    #torque_sim=real_torque_to_sim_torque(current_read)
    #print("torque sim",torque_read_sim,"\n")
    #print("torque real",torque_sim,"\n")
    #print("current",current_read,"\n")
    while (time.time()-start_time_t)*1000<10.00:
        1
    end_time_t=time.time()
    print("last time",end_time_t-start_time_t)
    
    #print("Press any key to continue! (or press ESC to escape)")
    #if getch() != chr(0x1b):
        #servos.write_some_positions(angles_real[0:6],DXLn_ID)
        #servos.write_all_positions_angles(angles_real)
    #time.sleep(0.07)
    if step==T-1:
        step=0
    else:
        step=step+1
        
    
    
    
str1="pos_0_5_10_1"+".json"
print(str1)
data = {'positions_tick': positions_tick,'current_pos_tick':current_pos_tick,'goal_pos_sim':goal_pos_sim,'phase':phase_all,}
data_json = json.dumps(data, cls=NumpyArrayEncoder)
with open(str1, 'w') as f:
    json.dump(data, f, cls=NumpyArrayEncoder)
    print("save data done!")
print("Press any key to disable legs")
        