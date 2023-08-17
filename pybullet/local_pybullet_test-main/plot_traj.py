import csv
import matplotlib.pyplot as plt

def get_current_pos_and_goal(row):
    current_pos=[]
    goal_pos=[]
    for i in range(18):
        current_pos.append(eval(row[6*i+1]))
        goal_pos.append(eval(row[6*i+2]))
    return current_pos,goal_pos


obstacle_current_traj=[[] for i_1 in range(18)]
obstacle_goal_traj=[[] for i_1 in range(18)]
cpg_goal_traj=[[] for i_1 in range(18)]
cpg_current_traj=[[] for i_1 in range(18)]

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
with open('/home/xu-ye/Downloads/dynamixel_workbench_master-main(1)/dynamixel_workbench_master-main/dynamixel_workbench_toolbox/test/build/servo_all_fre_cpg_1_2.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count=0
        for row in csv_reader:
            if line_count>0:
                current_pos,goal_pos=get_current_pos_and_goal(row)
                for i in range(18):
                    cpg_current_traj[i].append(current_pos[i])
                    cpg_goal_traj[i].append([goal_pos[i]])
            line_count+=1

plt.figure(1)
length=len(obstacle_current_traj[0])
for i in range(6):
    plt.subplot(3,2,i+1)
    plt.plot(range(length),obstacle_goal_traj[3*i],'b',range(length),obstacle_current_traj[3*i],'r',range(length),cpg_current_traj[3*i],'g',)
plt.show()
