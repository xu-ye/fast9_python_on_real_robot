import csv
import matplotlib.pyplot as plt
import numpy as np
import json


with open('pos_0_5_8.json', 'r') as f:
    
    data_read = json.load(f)
    positions_tick = np.asarray(data_read['positions_tick'])
    current_pos_tick = np.asarray(data_read['current_pos_tick'])
    goal_pos_sim = np.asarray(data_read['goal_pos_sim'])
    phase = np.asarray(data_read['phase'])


with open('pos_0_5_8_drop1.json', 'r') as f:
    
    data_read1 = json.load(f)
    positions_tick_drop = np.asarray(data_read1['positions_tick'])
    current_pos_tick_drop = np.asarray(data_read1['current_pos_tick'])
    goal_pos_sim_drop = np.asarray(data_read1['goal_pos_sim'])
    phase_drop = np.asarray(data_read1['phase'])
offset=0
length=len(current_pos_tick)
for i in range(6):
    plt.subplot(3,2,i+1)
    plt.plot(range(length),current_pos_tick[:,3*i],'b',range(length),current_pos_tick_drop[:,3*i],'r',range(length),positions_tick_drop[:,3*i],'g',)
plt.show()
for i in range(3):
    plt.subplot(1,3,i+1)
    plt.plot(range(length),current_pos_tick[:,offset+i],'b',range(length),current_pos_tick_drop[:,offset+i],'r',range(length),positions_tick_drop[:,offset+i],'g',)
plt.show()
