from Servos import *
import numpy as np

from CPGs import *
import time
import matplotlib.pyplot as plt

from Servos import *

servos = Servos()
DXLn_ID = range(18)
print(DXLn_ID)
ID = 0
servos.read_voltage(ID)
position_Read = servos.read_all_positions()
print("read position:", position_Read)
servos.set_current_based_control()
position_Read = servos.read_all_positions()
print("read position:", position_Read)

print("goal current:", servos.read_goal_current())

print("Press any key to enable legs")
if getch() != chr(0x1b):
    servos.enable_torque(DXLn_ID)

goal_position = np.array([180, 205, 103, 180, 205, 103, 180, 205, 103, 180, 205, 103, 180, 205, 103, 180, 205, 103])
servos.Robot_initialize(goal_position)

with open('cpg_fre_amp1.json', 'r') as f:
    data_read = json.load(f)
    thetan = np.asarray(data_read['joint_angles'])
    zzn0 = np.asarray(data_read['CPG_output'])

theta0=servos.Robot_initialize(thetan[0,:,0])
n_try=np.shape(thetan)[2]
length_per_try=np.shape(thetan)[0]
for index_try in range(n_try):
    positions=[]
    
    velocitys=[]
    currents=[]
    for index_l in range(length_per_try):
        if index_l%1==0:
            start_T = time.time()
            theta1 = thetan[index_l,:,index_try]
            position_goal = np.trunc(theta1 / 360 * 4096)  # tick

            servos.write_all_positions(position_goal)
            # 18
            dxl_present_position=servos.read_all_positions_tick()
            #dxl_present_velocity=servos.read_all_velocity()
            #dxl_present_current=servos.read_all_current()
            #dxl_present_position, dxl_present_velocity, dxl_present_current = servos.read_all_positions_velocity_current()
            positions.append(dxl_present_position)
            #velocitys.append(dxl_present_velocity)
            #currents.append(dxl_present_current)
            # time.sleep(dt/2)
            #print("position",dxl_present_position)
            while(time.time()-start_T<0.0010):
                time.sleep(0.00001)
                


            end_T = time.time()
            print("move:", end_T - start_T)
    positions=np.vstack(positions)
    #currents=np.vstack(currents)
    #velocitys=np.vstack(velocitys)
    str1="data4_p_"+str(index_try)+".json"
    print(str1)
    data = {'positions': positions, 'currents': currents,"velocitys":velocitys}
    data_json = json.dumps(data, cls=NumpyArrayEncoder)
    with open(str1, 'w') as f:
        json.dump(data, f, cls=NumpyArrayEncoder)
        print("save data done!")
    if index_try%50==0:
        
        print("press to  escape  to continue next try")
        if getch() == chr(0x1b):
            continue
        else:
            break

print("Press any key to disable legs")
if getch() != chr(0x1b):
    servos.disable_torque(DXLn_ID)
