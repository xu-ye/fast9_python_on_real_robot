import csv
#import matplotlib.pyplot as plt
import numpy as np

def get_current_pos_and_goal(row):
    current_pos=[]
    goal_pos=[]
    for i in range(18):
        current_pos.append(eval(row[6*i+1]))
        goal_pos.append(eval(row[6*i+2]))
    return current_pos,goal_pos

def split_string(string_o,num):
    sum_leg=[]
    split_list2=string_o.split(sep=None, maxsplit=-1)
    if len(split_list2)==num:
        for i in range(num):
            if i <(num-1) and i>0:
                sum_leg.append(eval(split_list2[i]))
            elif i==0:
                sum_leg.append(eval(split_list2[i][1:]))
            else:
                sum_leg.append(eval(split_list2[i][0:-1]))
    elif len(split_list2)==num+1:
        if string_o[1]==' ':
            for i in range(num):
                if i <(num-1):
                    sum_leg.append(eval(split_list2[i+1]))
                else:
                    sum_leg.append(eval(split_list2[i+1][0:-1]))
        else:
            for i in range(num):
                if i >0:
                    sum_leg.append(eval(split_list2[i]))
                else:
                    sum_leg.append(eval(split_list2[i][1:]))

    elif len(split_list2)==num+2:
        for i in range(num):
            
            sum_leg.append(eval(split_list2[i+1]))
    else:
        print("error")

        
        
    return sum_leg

def eval_row2(row):
    count=row[0]
    T_count=row[1]
    sum_leg=split_string(row[2],6)
    reflex_sim=split_string(row[3],6)
    reflex_stance_sim=split_string(row[4],6)
    on_reflex2=split_string(row[5],6)
    on_reflex_stance2=split_string(row[6],6)
    reflex_index2=split_string(row[7],6)
    reflex_index_stance2=split_string(row[8],6)
    swing_step_count=row[9]
    flat_cpg_tick=split_string(row[10],18)
    position_read_tick=split_string(row[11],18)
    IMU_data=split_string(row[12],9)
    #voltage=row[13]
    

    
    return count,T_count,sum_leg,reflex_sim,reflex_stance_sim,on_reflex2,on_reflex_stance2,\
reflex_index2,reflex_index_stance2,swing_step_count,flat_cpg_tick,position_read_tick,IMU_data
    