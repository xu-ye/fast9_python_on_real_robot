import threading
import time
from queue import Queue
import datetime

from multiprocessing import Process,Queue
import numpy as np
from CPGs import *
import time
import matplotlib.pyplot as plt

from Servos import *


with open('cpg_fre_amp1.json', 'r') as f:
    data_read = json.load(f)
    thetan = np.asarray(data_read['joint_angles'])
    zzn0 = np.asarray(data_read['CPG_output'])
    

servos=Servos()
DXLn_ID=range(18)
print(DXLn_ID)
ID=0
servos.read_voltage(ID)
position_Read=servos.read_all_positions()
print("read position:",position_Read)
servos.set_current_based_control()
position_Read=servos.read_all_positions()
print("read position:",position_Read)

print("goal current:",servos.read_goal_current())

print("Press any key to enable legs")
if getch() != chr(0x1b):
    servos.enable_torque(DXLn_ID)

goal_position=np.array([180,205,103,180,205,103,180,205,103,180,205,103,180,205,103,180,205,103])
servos.Robot_initialize(goal_position)
state_t = 0
amp = 0.35
amp2 = 0.15
fre = 0.05
df = 0.3
PH = [0, 0.5, 0.5, 0, 0, 0.5]  # 按照真实机器人上的腿排列顺序来的01 右左第一条腿
dt = 1.0 / 240.0
theta_phase = [[0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0], [0, 0.5, 0, 0.5, 0, 0.5],
               [-0.5, 0, -0.5, 0, -0.5, 0],
               [0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0]]
for j in range(6):
    for i in range(6):
        theta_phase[i][j] = PH[i] - PH[j]

# 计算初始位置并且设置

z0 = np.ones(12) * 0.5
cpgs = CPGs_O(theta_phase, dt, 0.5)

zz0 = cpgs.cpg_begin_all(z0)
theta0 = cpgs.cpg2theta_RealRobot(zz0)  # 角度
print("theta0",theta0)

servos.Robot_initialize(theta0)



print("时间：", 0, "位置：", theta0)

tf = 5
start_T = time.time()
zz_n = cpgs.two_layer_out_all(zz0, 0, tf)
t_eval = np.arange(0.0, tf, dt)
end_T = time.time()
print(end_T - start_T)

print("end cal")

count = 0

com_n = np.zeros((1, 3))
com_n[0, 2] = 0.1
theta1_n = np.zeros((1, 18))
print(servos.read_all_current())
positions=[]
currents=[]

print("Press any key to continue move")
if getch() != chr(0x1b):
    for t in t_eval:
        
        if count%1==0:
            zz1 = zz_n[:, :, count]
            start_T = time.time()
            cpgs.old_zz = zz1

            theta1 = cpgs.cpg2theta_RealRobot(zz_n[:, :, count])
            
            position_goal = np.trunc(theta1 / 360 * 4096)  # tick
        
        #end_T = time.time()
        #print("cal:",end_T - start_T)
        #start_T = time.time()
        
            servos.write_all_positions(position_goal)
        #time.sleep(dt/2)
           # position_Read=servos.read_all_positions()
            #print("read position:",position_Read)
            dxl_present_position=servos.read_all_positions_tick()
            #dxl_present_velocity=servos.read_all_velocity()
            #dxl_present_current=servos.read_all_current()
            #dxl_present_position, dxl_present_velocity, dxl_present_current = servos.read_all_positions_velocity_current()
            positions.append(dxl_present_position)
            #dxl_present_current=servos.read_all_current()
            #currents.append(dxl_present_current)
            
        
            #servos.read_voltage(ID)
            #servos.read_current_single(2)
            #servos.read_current_single(4)
            #print(servos.read_all_current())
            while(time.time()-start_T<0.002):
                1
                
            
            
            end_T = time.time()
            print("move:",end_T - start_T)
        #for index_leg in range(1):
            #servos.read_current_2legs(index_leg)
        #servos.read_current_2legs(0)

        #print("时间：", t, "位置：", theta1)
        count = count+1
        
str1="positions_pure_cpg"+".json"
print(str1)
data = {'positions': positions}
data_json = json.dumps(data, cls=NumpyArrayEncoder)
with open(str1, 'w') as f:
    json.dump(data, f, cls=NumpyArrayEncoder)
    print("save data done!")
print("Press any key to disable legs")
if getch() != chr(0x1b):
    servos.disable_torque(DXLn_ID)



