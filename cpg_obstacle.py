#coding=GBK”

import json
from sys import path
path.append("../../")
import math
import numpy as np
import random
import csv
#import Servos
import copy


class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)



def get_reflex_theta(theta_original,coef,reflex,on_reflex,phase,):
    # 当且仅当为reflex 并且 摇摆相 并且在reflex 附近的一个cpg周期内
    #phase -1 支撑相  1 摇摆相
    # 输出  返回考虑reflex后的一个目标值（仿真中的目标值）
    cf_base=0.3
    ft_base=-0.33
    theta_new=np.zeros_like(theta_original)
    for j in range(6):
        if (reflex[j] or on_reflex[j]) and phase[j]==1:
            theta_new[j, 0] = 1*theta_original[j,0]
            if j < 3:
                theta_new[j,1]=(theta_original[j,1]+cf_base)*coef-cf_base
                theta_new[j,2]=(theta_original[j,2]+ft_base)*coef-ft_base

            else:
                theta_new[j,1]=(theta_original[j,1]-cf_base)*coef+cf_base
                theta_new[j,2]=(theta_original[j,2]-ft_base)*coef+ft_base
        else:
            theta_new[j]=theta_original[j]
    return  theta_new   

def get_reflex_theta_stance(theta_original,coef,reflex_stance,on_reflex_stance,phase,):
    # 当且仅当为reflex 并且 摇摆相 并且在reflex 附近的一个cpg周期内
    #phase -1 支撑相  1 摇摆相
    # 输出  返回考虑reflex后的一个目标值（仿真中的目标值）
    cf_base=0.3
    ft_base=-0.33
    theta_new=np.zeros_like(theta_original)
    for j in range(6):
        if (reflex_stance[j] or on_reflex_stance[j]) and phase[j]==-1:
            
            theta_new[j, 0] = 1*theta_original[j,0]
            theta_new[j,1]=theta_original[j,1]*coef
            theta_new[j,2]=theta_original[j,2]*coef
            
        else:
            theta_new[j]=theta_original[j]
    return  theta_new 


def get_reflex_theta_all(theta_original,coef,coef_stance,reflex,on_reflex,reflex_stance,on_reflex_stance,phase,stance_step_per_reflex,swing_step_per_reflex):
    # 当且仅当为reflex 并且 摇摆相 并且在reflex 附近的一个cpg周期内
    #phase -1 支撑相  1 摇摆相
    # 输出  返回考虑reflex后的一个目标值（仿真中的目标值）
    cf_base=0.3
    ft_base=-0.33
    theta_new=np.zeros_like(theta_original)
    for j in range(6):
        
        if (reflex[j] or on_reflex[j]) and phase[j]==1:
            coef_stance_now=1
            if on_reflex_stance[j]==1:# 说明先支撑再摇摆
                if swing_step_per_reflex[j]<20:
                    coef_stance_now=-(coef_stance-1)*(swing_step_per_reflex[j])/20+coef_stance
    
            
                    
            theta_new[j, 0] = 1*theta_original[j,0]
            coef_stance_now=1
            if j < 3:
                theta_new[j,1]=(theta_original[j,1]+cf_base)*coef-cf_base*coef_stance_now
                theta_new[j,2]=(theta_original[j,2]+ft_base)*coef-ft_base*coef_stance_now

            else:
                theta_new[j,1]=(theta_original[j,1]-cf_base)*coef+cf_base*coef_stance_now
                theta_new[j,2]=(theta_original[j,2]-ft_base)*coef+ft_base*coef_stance_now
        elif (reflex_stance[j] or on_reflex_stance[j]) and phase[j]==-1: 
            if stance_step_per_reflex[j]<20:
                coef_stance_now=(coef_stance-1)*(stance_step_per_reflex[j])/20+1
            else:
                coef_stance_now=coef_stance
            #coef_stance_now=coef_stance
            theta_new[j, 0] = 1*theta_original[j,0]
            theta_new[j,1]=theta_original[j,1]*coef_stance_now
            theta_new[j,2]=theta_original[j,2]*coef_stance_now
            
        else:
            theta_new[j]=theta_original[j]
    return  theta_new 

def judge_reflex(traj_error):
    # traj_error (4,18)
    flag=np.ones((18)) # same direction 1
    
    for j in range(18):
        for i in range(3):
            if traj_error[3][j]*traj_error[i][j]<=0:
                flag[j]=0
                break
            #if abs(traj_error[i][j])>abs(traj_error[i+1][j]):
                #flag[j]=0
                #break
            # 考虑是否加上最后一项 大于前面三项
    sum_error=np.sum(traj_error,axis=0)
    sum_flag=abs((sum_error*flag.transpose())).reshape((6,3))
    sum_leg=np.sum(sum_flag,axis=1)
    #print("sum leg",sum_leg)
    reflex=np.zeros(6)
    reflex[sum_leg>90]=1
    return reflex,sum_leg

def judge_reflex_stance(traj_error):
    # traj_error (4,18)
    flag=np.ones((18)) # same direction 1
    
    for j in range(18):
        for i in range(3):
            if traj_error[3][j]*traj_error[i][j]<=0:
                flag[j]=0
                break
            #if abs(traj_error[i][j])>abs(traj_error[i+1][j]):
                #flag[j]=0
                #break
    sum_error=np.sum(traj_error,axis=0)
    sum_flag=abs((sum_error*flag.transpose())).reshape((6,3))
    sum_leg=np.sum(sum_flag,axis=1)
    
    reflex=np.zeros(6)
    reflex[sum_leg>100]=1
    return reflex,sum_leg


def sim_angles_to_real0(theta):
    # theta [6,3]
    real_angles=np.zeros_like(theta).flatten()
    for i in range(6):
        if i<3:
            
            hip=180+theta[i,0]/math.pi*180 #鍚戝墠涓哄?炲??
            
            knee=180-theta[i,1]/math.pi*180
            ankle=62.69+theta[i,2]/math.pi*180
            
            real_angles[3+i*6+0]=hip
            real_angles[3+i*6+1]=knee
            real_angles[3+i*6+2]=ankle
            
        else:
            hip=180+theta[i,0]/math.pi*180 #鍚戝墠涓哄?炲??
            knee=180+theta[i,1]/math.pi*180
            ankle=62.69-theta[i,2]/math.pi*180
            
            real_angles[0+(i-3)*6+0]=hip
            real_angles[0+(i-3)*6+1]=knee
            real_angles[0+(i-3)*6+2]=ankle
        
    return real_angles

def angles_to_tick(angles):
    theta_tick=np.zeros_like(angles)
    for i in range(18):
        theta_tick[i]=int(angles[i]/180*1.0*2048)
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
    # theta [6,3]  杈撳嚭 np array [18]
    real_angles=np.zeros_like(theta).flatten()*1.00000
    for i in range(6):
        if i<3:
            
            hip=(180-theta[i,0]/math.pi*180) #鍚戝墠涓哄?炲??
            
            knee=180-theta[i,1]/math.pi*180
            ankle=62.69+theta[i,2]/math.pi*180
            
            real_angles[0+i*6+0]=hip
            real_angles[0+i*6+1]=knee
            real_angles[0+i*6+2]=ankle
            
        else:
            hip=(180-theta[i,0]/math.pi*180) #鍚戝墠涓哄?炲??
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


def real_reflex_leg_to_sim(reflex_real):
    # reflex_real (6,1)
    reflex_sim=np.zeros(6)
    reflex_sim[0]=reflex_real[0]
    reflex_sim[1]=reflex_real[2]
    reflex_sim[2]=reflex_real[4]
    reflex_sim[3]=reflex_real[1]
    reflex_sim[4]=reflex_real[3]
    reflex_sim[5]=reflex_real[5]
    return reflex_sim


def sim_reflex_leg_to_real(reflex_sim):
    # reflex_real (6,1)
    reflex_real=np.zeros(6)
   
    reflex_real[0]=reflex_sim[0]
    reflex_real[2]=reflex_sim[1]
    reflex_real[4]=reflex_sim[2]
    reflex_real[1]=reflex_sim[3]
    reflex_real[3]=reflex_sim[4]
    reflex_real[5]=reflex_sim[5]
    return reflex_real

def swing_reflex(reflex_sim,phase):
    new_reflex=np.zeros(6)
    for i in range(6):
        if phase[i]==1:
            new_reflex[i]=reflex_sim[i]
    return new_reflex

def stance_reflex(reflex_sim,phase):
    new_reflex=np.zeros(6)
    for i in range(6):
        if phase[i]==-1:
            new_reflex[i]=reflex_sim[i]
    return new_reflex

def stance_and_swing_reflex(reflex_sim_swing,reflex_sim_stance,phase):
    new_reflex=np.zeros(6)
    for i in range(6):
        if phase[i]==1:
            new_reflex[i]=reflex_sim_swing[i]
        else:
            new_reflex[i]=reflex_sim_stance[i]
            
    return new_reflex



with open('force_real7.json', 'r') as f:
    
    data_read = json.load(f)
    force_r = np.asarray(data_read['contact_force'])
    phase_r = np.asarray(data_read['phase'])
    cpg_r = np.asarray(data_read['cpg'])
    theta_r = np.asarray(data_read['theta'])
    theta_ini_r=np.asarray(data_read['theta_ini'])
    ini_index_r=np.asarray(data_read['ini_index'])
    torque_n=np.asarray(data_read['torque_n'])


with open('pos_0_5_10_1.json', 'r') as f:
    
    data_read = json.load(f)
    positions_tick = np.asarray(data_read['positions_tick'])
    current_pos_tick = np.asarray(data_read['current_pos_tick'])
    goal_pos_sim = np.asarray(data_read['goal_pos_sim'])
    phase = np.asarray(data_read['phase'])
    
step= 0
cpg_index=ini_index_r[0]


import numpy as np

from Servos import *


servos=Servos()
#servos.light_LED()
#goal_position=2048*np.ones(1,18)
servos.read_voltage(1
                    )
servos.set_position_control()
goal_position=np.array([180,204,85,180,204,85])
#position_Read=servos.read_position_loop()
position_all=range(18)
print("Press any key to enable legs! (or press ESC to escape!)")
if getch() != chr(0x1b):
    servos.enable_torque(position_all)

#print("read position:",position_Read)
'''
DXLn_ID=[0,1,2,3,4,5]
servos.enable_torque(DXLn_ID)
print("Press any key to continue! (or press ESC to move leg2!)")
if getch() == chr(0x1b):
    servos.write_some_positions(goal_position,DXLn_ID)
time.sleep(1)
DXLn_ID=[6,7,8,9,10,11]
servos.enable_torque(DXLn_ID)
print("Press any key to continue! (or press ESC to move leg2!)")
if getch() == chr(0x1b):
    servos.write_some_positions(goal_position,DXLn_ID)


DXLn_ID=[12,13,14,15,16,17]
servos.enable_torque(DXLn_ID)
print("Press any key to continue! (or press ESC to move leg2!)")
if getch() == chr(0x1b):
    servos.write_some_positions(goal_position,DXLn_ID)
'''
position_Read=servos.read_all_positions()
print("read position:",position_Read)



# initial
positions=[]
traj_error_buf=np.zeros((4,18))
on_reflex=np.zeros(6)
reflex_index=np.ones(6)*(-10)

on_reflex_stance=np.zeros(6)
reflex_index_stance=np.ones(6)*(-10)
stance_step_per_reflex=np.ones(6)*(0)
swing_step_per_reflex=np.ones(6)*(0)

step=0
theta_sim=goal_pos_sim[step]    
angles_real=sim_angles_to_real(theta_sim)
servos.Robot_initialize(angles_real)
goal_theta_tick=angles_to_tick(angles_real)
time.sleep(5)
position_Read=servos.read_all_positions()
position_Read_tick=angles_to_tick(position_Read)
flat_cpg_tick=current_pos_tick[step] #18
traj_error_buf[0]=flat_cpg_tick-position_Read_tick

theta_tick=angles_to_tick(angles_real)
servos.write_all_positions(theta_tick)
servos.write_all_positions(theta_tick)
servos.write_all_positions(theta_tick)


T=480 
T_count=0


coef=2
coef_stance=1.5
step=0
reflex=np.zeros(6)

sum_leg_all=[]
reflex_real_all=[]
on_reflex_all=[]
traj_error_all=[]
count_all=[]
csv_rows=[]

# 计数在每个swing or reflex中的步数
swing_step_count=0



# -0.3 0.33
for count in range(int(T*3)):
    start_time_t=time.time()
    
    
    
    
    if count<4:
        # just give command
        theta_sim=goal_pos_sim[count]    
        angles_real=sim_angles_to_real(theta_sim)
        theta_tick=angles_to_tick(angles_real)
        servos.write_all_positions(theta_tick)
        # read feedback
        position_Read=servos.read_all_positions()
        position_Read_tick=angles_to_tick(position_Read)
        flat_cpg_tick=current_pos_tick[count] #18
        traj_error_buf[count]=flat_cpg_tick-position_Read_tick
        on_reflex=np.zeros(6)
        
    else:
        # judhe whwther or not is a reflex
        # 在判断一开始的reflex 之前就清零
        phase_now=phase[step]
        last_phase=phase[step-1]
        
        
        reflex,sum_leg=judge_reflex(traj_error_buf)
        reflex_sim=real_reflex_leg_to_sim(reflex)
        
        reflex_sim=swing_reflex(reflex_sim,phase_now)
        reflex_stance,sum_leg_stance=judge_reflex_stance(traj_error_buf)
        reflex_stance_sim0=real_reflex_leg_to_sim(reflex_stance)
        reflex_stance_sim=stance_reflex(reflex_stance_sim0,phase_now)
        
        print("sum_leg",sum_leg,'sum_leg_stance',sum_leg_stance)
        
        #print("count:",count,"reflex",reflex,"on reflex",on_reflex)
        # on_reflex 在sim的基础上
        theta_sim=goal_pos_sim[step] 
        #on_reflex=swing_reflex(on_reflex,phase_now)
        #on_reflex_stance=stance_reflex(on_reflex_stance,phase_now)
        if reflex_sim.any():
            reflex_index[reflex_sim>0]=T_count
            on_reflex[reflex_sim>0]=1
            
        if reflex_stance_sim.any():
            reflex_index_stance[reflex_stance_sim>0]=T_count
            on_reflex_stance[reflex_stance_sim>0]=1
            on_reflex_stance[on_reflex>0]=0
        if on_reflex_stance.any():
            stance_step_per_reflex[on_reflex_stance>0]+=1
        if on_reflex_stance.any():
            swing_step_per_reflex[on_reflex>0]+=1
            
        swing_step_per_reflex[swing_step_per_reflex>239]=0
        stance_step_per_reflex[stance_step_per_reflex>239]=0
        # swing 和stance  交界处 error 清零
        swing_step_per_reflex[last_phase*phase_now<0]=0
        stance_step_per_reflex[last_phase*phase_now<0]=0
        swing_step_count+=1
        if (last_phase*phase_now)[0]<1:
            traj_error_buf=np.zeros_like(traj_error_buf)
            swing_step_count=0
            
            
            
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
        
        if swing_step_count<10:
            coef=1
            #coef_stance=1
        else:
            coef=2
            #coef_stance=1.5
        theta_new_sim=get_reflex_theta_all(theta_sim,coef,coef_stance,reflex_sim,on_reflex,reflex_stance_sim,on_reflex_stance,phase_now,stance_step_per_reflex,swing_step_per_reflex)
        angles_real=sim_angles_to_real(theta_new_sim)
        theta_tick=angles_to_tick(angles_real)
        servos.write_all_positions(theta_tick)
        
         # read feedback
        position_Read=servos.read_all_positions()
        position_Read_tick=angles_to_tick(position_Read)
        flat_cpg_tick=current_pos_tick[step] #18
        
        on_reflex_real=sim_reflex_leg_to_real(on_reflex)
        on_reflex_stance_real=sim_reflex_leg_to_real(on_reflex_stance)
        
        
        for i in range(6):
            if on_reflex_real[i]==1:
                flat_cpg_tick[3*i:3*i+3]=theta_tick[3*i:3*i+3]
            elif on_reflex_stance_real[i]==1:
                flat_cpg_tick[3*i:3*i+3]=theta_tick[3*i:3*i+3]
        
        
        ## buffer 升级
        for i_index in range(3):
            traj_error_buf[i_index]=traj_error_buf[i_index+1]
        traj_error_buf[3]=flat_cpg_tick-position_Read_tick
        
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
        
        
        
        csv_row=[]
        csv_row=[count,T_count,sum_leg,reflex_sim,reflex_stance_sim,on_reflex2,on_reflex_stance2,reflex_index2,reflex_index_stance2,swing_step_count,flat_cpg_tick-position_Read_tick]
        csv_rows.append(csv_row)
        
        
        
        
        
        
        
        
        
        
    
    #print("current",current_read,"\n")
        while (time.time()-start_time_t)*1000<10.00:
            1
        end_time_t=time.time()
        print("last time",time.time()-start_time_t,"count:",count,"reflex",reflex_sim,"on reflex",on_reflex,'reflex_index',reflex_index,'T_count',T_count)
        print("reflex_stance",reflex_stance_sim,"on reflex",on_reflex_stance,'reflex_index_stance',reflex_index_stance,'T_count',)
        
    
    
    
    
        
    
    
    if step==479:
        step=0
    else:
        step=step+1
    if step%T==int(T/4-1):
        T_count+=1

str1="positions_pure_"+".json"
print(str1)
# csv
with open('data_new_traj9.csv', mode='w', newline='') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(['count','T_count','sum_leg','reflex','on_reflex_','reflex_index','traj_error'])
    writer.writerows(csv_rows)

data = {'positions': positions}
data_json = json.dumps(data, cls=NumpyArrayEncoder)
'''
with open(str1, 'w') as f:
    json.dump(data, f, cls=NumpyArrayEncoder)
    print("save data done!")
print("Press any key to disable legs")
'''
        
