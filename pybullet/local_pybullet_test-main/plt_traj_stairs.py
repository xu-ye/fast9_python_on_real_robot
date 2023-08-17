import csv
import matplotlib.pyplot as plt
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


def eval_row(row):
    count=row[0]
    T_count=row[1]
    sum_leg=[]
    reflex_sim=[]
    reflex_stance_sim=[]
    on_reflex2=[]
    on_reflex_stance2=[]
    reflex_index2=[]
    reflex_index_stance2=[]
    swing_step_count=row[9]
    flat_cpg_tick=[]
    position_read_tick=[]
    IMU_data=[]
    voltage=row[13]
    split_list2=row[2].split(sep=None, maxsplit=-1)
    if len(split_list2)==6:
        for i in range(6):
            if i <5 and i>0:
                sum_leg.append(eval(split_list2[i]))
            elif i==0:
                sum_leg.append(eval(split_list2[i][1:]))
            else:
                sum_leg.append(eval(split_list2[i][0:-1]))
    elif len(split_list2)==7:
        for i in range(6):
            if i <5:
                sum_leg.append(eval(split_list2[i+1]))
            else:
                sum_leg.append(eval(split_list2[i+1][0:-1]))

    sum_leg1=split_string(row[2],6)
    for i in range(6):
        #sum_leg.append(eval(row[2][1+4*i:3+4*i]))
        reflex_sim.append(eval(row[3][1+3*i:3+3*i]))
        reflex_stance_sim.append(eval(row[4][1+3*i:3+3*i]))
        on_reflex2.append(eval(row[5][1+3*i:3+3*i]))
        on_reflex_stance2.append(eval(row[6][1+3*i:3+3*i]))
        reflex_index2.append(eval(row[7][1+5*i:5+5*i]))
        reflex_index_stance2.append(eval(row[8][1+5*i:5+5*i]))


    for i in range(18):
           if i<11:
                flat_cpg_tick.append(eval(row[10][1+6*i:6+6*i]))
                position_read_tick.append(eval(row[11][1+6*i:6+6*i]))
           else:
                flat_cpg_tick.append(eval(row[10][1+6*i+1:6+6*i+1]))
                position_read_tick.append(eval(row[11][1+6*i+1:6+6*i+1]))
    
    split_list=row[12].split(sep=None, maxsplit=-1)
    if len(split_list)==10:
        for i in range(9):
            if i <8:
                IMU_data.append(eval(split_list[i+1]))
            else:
                IMU_data.append(eval(split_list[i+1][0:-1]))

    '''    
    for i in range(9):
        if len(row[12])==83:
            if i<8:
                IMU_data.append(eval(row[12][1+9*i:9+9*i]))
            else:
                IMU_data.append(eval(row[12][1+9*i+1:9+9*i+1]))
        else:
            IMU_data.append(eval(row[12][1+12*i:12+12*i]))
            '''
    return count,T_count,sum_leg,reflex_sim,reflex_stance_sim,on_reflex2,on_reflex_stance2,\
reflex_index2,reflex_index_stance2,swing_step_count,flat_cpg_tick,position_read_tick,IMU_data,voltage


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
    
         

                





if __name__ == '__main__':

    obstacle_current_traj=[[] for i_1 in range(18)]
    obstacle_goal_traj=[[] for i_1 in range(18)]
    cpg_goal_traj=[[] for i_1 in range(18)]
    cpg_current_traj=[[] for i_1 in range(18)]
    IMU_data_all=[]
    flat_cpg_tick_all=[]
    position_read_tick_all=[]


    with open('/home/xu-ye/Downloads/dynamixel_workbench_master-main(1)/dynamixel_workbench_master-main/dynamixel_workbench_toolbox/test/build/servo_all_fre_cpg_obstable_16_10_5_1_5_2.csv') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count=0
            for row in csv_reader:
                if line_count>0:
                    current_pos,goal_pos=get_current_pos_and_goal(row)
                    for i in range(18):
                        obstacle_current_traj[i].append(current_pos[i])
                        obstacle_goal_traj[i].append([goal_pos[i]])
                line_count+=1
    with open('/home/xu-ye/Downloads/servo_python-main/data_reflex_imu_stairs_2_0.csv') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count=0
            for row in csv_reader:
                if line_count>0:
                    #current_pos,goal_pos=get_current_pos_and_goal(row)
                    
                    count,T_count,sum_leg,reflex_sim,reflex_stance_sim,on_reflex2,on_reflex_stance2,\
    reflex_index2,reflex_index_stance2,swing_step_count,flat_cpg_tick,position_read_tick,IMU_data=eval_row2(row)
                    IMU_data_all.append(IMU_data)
                    flat_cpg_tick_all.append(flat_cpg_tick)
                    position_read_tick_all.append(position_read_tick)

                        #cpg_current_traj[i].append(current_pos[i])
                        #cpg_goal_traj[i].append([goal_pos[i]])
                line_count+=1

    IMU_data_all=np.array(IMU_data_all)
    flat_cpg_tick_all=np.array(flat_cpg_tick_all)
    position_read_tick_all=np.array(position_read_tick_all)
    plt.figure(1)
    length=IMU_data_all.shape[0]
    for i in range(9):
        plt.subplot(3,3,i+1)
        plt.plot(range(length),IMU_data_all[:,i],'b',)
    plt.show()


    plt.figure(1)

    for i in range(6):
        plt.subplot(3,2,i+1)
        plt.plot(range(length),flat_cpg_tick_all[:,3*i+1],'b',range(length),position_read_tick_all[:,3*i+1],'r',)
    plt.show()

    plt.figure(1)

