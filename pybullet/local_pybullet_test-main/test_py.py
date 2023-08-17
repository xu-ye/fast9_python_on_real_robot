import multiprocessing

import numpy as np
from scipy.integrate import solve_ivp
from scipy.integrate import odeint

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data as pd
from time import time,sleep
import math
import json
import tensorflow as tf
from CPGs import *
from multiprocessing import Process, Pipe, Queue

def write(q):
    print('子进程发送消息：')





    PH = [0, 1 / 3, 2 / 3, 2 / 3, 0, 1 / 3]
    theta_phase = [[0, 0.5, 0.25, 0, 0.5, 0.75], [-0.5, 0, -0.25, -0.5, 0, 0.25],
                   [-0.25, 0.25, 0, -0.25, 0.25, 0.5], [0, 0.5, 0.25, 0, 0.5, 0.75],
                   [-0.5, 0, -0.25, -0.5, 0, 0.25], [-0.75, -0.25, -0.5, -0.75, -0.25, 0]]
    for j in range(6):
        for i in range(6):
            theta_phase[i][j] = PH[i] - PH[j]
    dt = 1.0 / 240.0
    b = 0.5
    # 计算初始位置并且设置
    z0 = np.ones(12) * 0.5
    cpgs = CPGs_O(theta_phase, dt, b)
    zz0 = np.ones((6, 6)) * 0.5
    tf = 0.5
    start_T = time()
    zz_n = cpgs.two_layer_out_all(zz0, 0, tf)
    t_eval = np.arange(0.0, tf, dt)
    end_T = time()
    q.put(zz_n[:,-1,-1])
    print(end_T - start_T)
    print("put zz_n")
    q.put("he")
    print(time(), "put_another")



    print('子进程接受消息：')

    print(time(),"receive")
    zz_n = cpgs.two_layer_out_all(zz0, 0, 2)

    q.put(zz_n[-1,:,-1])
    print(time(),"put 3")



if __name__ == '__main__':
    PH = [0, 1 / 3, 2 / 3, 2 / 3, 0, 1 / 3]
    theta_phase = [[0, 0.5, 0.25, 0, 0.5, 0.75], [-0.5, 0, -0.25, -0.5, 0, 0.25],
                   [-0.25, 0.25, 0, -0.25, 0.25, 0.5], [0, 0.5, 0.25, 0, 0.5, 0.75],
                   [-0.5, 0, -0.25, -0.5, 0, 0.25], [-0.75, -0.25, -0.5, -0.75, -0.25, 0]]
    for j in range(6):
        for i in range(6):
            theta_phase[i][j] = PH[i] - PH[j]


    class NumpyArrayEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            return json.JSONEncoder.default(self, obj)


    conn1, conn2 = Pipe()  # 关键点，pipe实例化生成一个双向管
    print( tf.__version__)
    theta_q = Queue()

    p = Process(target=write, args=(theta_q,))  # conn2传给子进程
    p.start()
    print("主进程开始")


    theta_get=theta_q.get()
    print("get",theta_get)
    print('主进程接受消息：')

    theta_get2 = theta_q.get()
    print('主进程接收消息１：')
    print("get2", theta_get2)

    print(time())
    print('get3',theta_q.get())
    p.join()
    print('结束测试')


    PH = [0, 1/3, 2/3, 2/3, 0, 1/3]
    theta_phase = [[0, 0.5, 0.25, 0, 0.5, 0.75],[-0.5, 0, -0.25, -0.5, 0, 0.25],
                   [-0.25, 0.25, 0, -0.25, 0.25, 0.5],[0, 0.5, 0.25, 0, 0.5, 0.75],
                   [-0.5, 0, -0.25, -0.5, 0, 0.25],[-0.75, -0.25, -0.5, -0.75, -0.25, 0]]
    for j in range(6):
        for i in range(6):
            theta_phase[i][j]=PH[i]-PH[j]



    state_t = 0

    dt = 1.0 / 240.0
    b=0.5
    # 计算初始位置并且设置
    z0 = np.ones(12) * 0.5
    cpgs = CPGs_O(theta_phase, dt,b)
    zz0 = np.ones((6,6)) * 0.5
    tf=1
    start_T=time()
    zz_n = cpgs.two_layer_out_all(zz0, 0, tf)
    t_eval = np.arange(0.0, tf, dt)
    end_T=time()
    print(end_T-start_T)

    array1 = np.zeros((4, 5))
    data = {'joint_angles': array1}
    data_json = json.dumps(data, cls=NumpyArrayEncoder)
    print(f'data_json:{data_json}')

    # 写入json文件
    with open('array_json.json', 'w') as f:
        json.dump(data, f, cls=NumpyArrayEncoder)


    t = [[0]]
    position = [[0]]
    velocity = [[0]]
    force_z = [[0]]

    for j in range(18):
        position.append([0])
        velocity.append([0])
        force_z.append([0])
        t.append([0])

    print(position)
    position[1].append(1.1)
    position[0].append(2.2)
    t[0].append(1)
    plt.figure(1)
    plt.plot(t[0],position[0],'-r')
    plt.show()
    print(position)



    start_t1=time()
