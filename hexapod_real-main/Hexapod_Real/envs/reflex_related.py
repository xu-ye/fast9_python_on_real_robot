import json
from sys import path
path.append("../../")
import math
import numpy as np
import random
import csv
#import Servos
import copy

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
    # coef  1x6
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
                theta_new[j,1]=(theta_original[j,1]+cf_base)*coef[j]-cf_base*coef_stance_now
                theta_new[j,2]=(theta_original[j,2]+ft_base)*coef[j]-ft_base*coef_stance_now

            else:
                theta_new[j,1]=(theta_original[j,1]-cf_base)*coef[j]+cf_base*coef_stance_now
                theta_new[j,2]=(theta_original[j,2]-ft_base)*coef[j]+ft_base*coef_stance_now
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


def judge_new_reflex(reflex,swing_reflex_index):
    new_reflex=np.zeros_like(reflex)
    for i in range(6):
        if reflex[i]==1 and swing_reflex_index[i]>5:
            new_reflex[i]=1
    return new_reflex



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
    sum_leg_tick=sum_leg/1
    #print("sum leg",sum_leg)
    reflex=np.zeros(6)
    reflex[sum_leg_tick>70]=1
    return reflex,sum_leg_tick

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
    sum_leg_tick=sum_leg/1
    reflex=np.zeros(6)
    reflex[sum_leg_tick>100]=1
    return reflex,sum_leg_tick




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


def judge_reflex_force(force_l):
    reflex=np.zeros(6)
    for i in range(6):
        if force_l[i]>1e-1:
            reflex[i]=1
    return reflex


def judge_imu_reflex(imu_data):
    # 输入imu__data 弧度
    imu_data1=imu_data/3.1415926*180
    reflex=np.zeros(3)
    if abs(imu_data1[0])>3:
        reflex[0]=1
    if abs(imu_data1[1])>3:
        reflex[1]=1
    if abs(imu_data1[2])>5:
        reflex[2]=1
    return reflex

def judge_imu_reflex2(imu_data,phase):
    # 输入imu__data 弧度
    imu_data1=imu_data/3.1415926*180
    reflex=np.zeros(6)
    
    if abs(imu_data1[0])>3 or abs(imu_data1[1])>3 or abs(imu_data1[2])>5:
        for i in range(6):
            if phase[i]==-1:
                reflex[i]=1
    return reflex


def imu_roll_reflex(imu_reflex,phase,theta_original,coef):
    

    cf_base=0.3
    ft_base=-0.33
    theta_new=np.zeros_like(theta_original)
    for j in range(6):
        # imu 并且是支撑相
        if (imu_reflex[0] ) and phase[j]==-1:
            theta_new[j,0]=theta_original[j,0]
            
            
            if j < 3:
                theta_new[j,1]=(theta_original[j,1]+cf_base)*coef[j]-cf_base
                theta_new[j,2]=-(theta_original[j,1]+cf_base)*coef[j]-ft_base

            else:
                theta_new[j,1]=(theta_original[j,1]-cf_base)*coef[j]+cf_base
                theta_new[j,2]=-(theta_original[j,1]-cf_base)*coef[j]+ft_base
            
        else:
            theta_new[j]=theta_original[j]
    return  theta_new

def imu_roll_reflex2(imu_reflex,phase,theta_original,coef):
    

    cf_base=0.3
    ft_base=-0.33
    theta_new=np.zeros_like(theta_original)
    for j in range(6):
        # imu 并且是支撑相
        if (imu_reflex[j]==1 ) and phase[j]==-1:
            theta_new[j,0]=theta_original[j,0]
            
            
            if j < 3:
                theta_new[j,1]=(theta_original[j,1]+cf_base)*coef[j]-cf_base
                theta_new[j,2]=-(theta_original[j,1]+cf_base)*coef[j]-ft_base

            else:
                theta_new[j,1]=(theta_original[j,1]-cf_base)*coef[j]+cf_base
                theta_new[j,2]=-(theta_original[j,1]-cf_base)*coef[j]+ft_base
            
        else:
            theta_new[j]=theta_original[j]
    return  theta_new


def get_imu_coef(imu_data,imu_reflex):
    imu_data1=imu_data/3.1415926*180
    coef_imu=np.zeros(6)
    max_bound=100
    coef1=3
    #0.01*0.15*20=0.03
    if imu_data1[0]>3:
        coef_imu[2]=min(imu_data1[0]*coef1,max_bound)
        coef_imu[5]=min(imu_data1[0]*coef1,max_bound)
    if imu_data1[0]<-3:
        coef_imu[0]=min(-imu_data1[0]*coef1,max_bound)
        coef_imu[3]=min(-imu_data1[0]*coef1,max_bound)
    return coef_imu



