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
