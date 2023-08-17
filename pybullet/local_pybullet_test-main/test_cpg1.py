import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.integrate import odeint

import time

plt.style.use('seaborn-poster')


def test_fun(t, x):
    dx = np.cos(t)
    return dx


a = 100.0
b = 0.5  # 占空比
w_a = 4 * np.pi
w_stance = b * w_a
w_swing = (1 - b) * w_a
k = 4  # 不同腿之间的扩散系数
k2 = 2
p = 100  # 收敛速度
u = 1
dt = 1/240
theta = [[0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0], [0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0],
         [0, 0.5, 0, 0.5, 0, 0.5], [-0.5, 0, -0.5, 0, -0.5, 0]]
dzdt = np.zeros([12])
theta1 = [[0,0,0],[1/4, 0, 0], [1/4, 0, 0]]
for i in range(6):
    for j in range(6):
        theta[i][j] = theta[i][j] * 2 * np.pi

for i in range(3):
    for j in range(3):
        theta1[i][j] = theta1[i][j] * 2 * np.pi



# F = lambda t, s: np.cos(t)

def hopf6_3(z, t):
    for i in range(6):
        w = w_stance / (np.exp(-a * z[2 * i + 1]) + 1) + w_swing / (np.exp(a * z[2 * i + 1]) + 1)
        dzdt[2 * i] = p * (u - pow(z[2 * i + 1], 2) - pow(z[2 * i], 2)) * z[2 * i] - w * z[2 * i + 1]
        dzdt[2 * i + 1] = p * (u - pow(z[2 * i + 1], 2) - pow(z[2 * i], 2)) * z[2 * i + 1] + w * z[2 * i]
        for j in range(6):
            dzdt[2 * i] += k * (np.cos(theta[i][j]) * z[2 * j] - np.sin(theta[i][j]) * z[2 * j + 1])
            dzdt[2 * i + 1] += k * (np.sin(theta[i][j]) * z[2 * j] + np.cos(theta[i][j]) * z[2 * j + 1])
    return dzdt

def sec_hopf(z1):
    dz1=np.zeros([6, 6])
    for j in range(6): # 腿编号
        for i in [1,2]:
            w = w_stance / (np.exp(-a * z1[j,2 * i + 1]) + 1) + w_swing / (np.exp(a * z1[j,2 * i + 1]) + 1)
            dz1[j,2 * i] = p * (u - pow(z1[j,2 * i + 1], 2) - pow(z1[j,2 * i], 2)) * z1[j,2 * i] - w * z1[j,2 * i + 1]
            dz1[j,2 * i + 1] = p * (u - pow(z1[j,2 * i + 1], 2) - pow(z1[j,2 * i], 2)) * z1[j,2 * i + 1] + w * z1[j,2 * i]
            for k in range(3):
                dz1[j,2 * i] += k * (np.cos(theta1[i][k]) * z1[j,2 * k] - np.sin(theta1[i][k]) * z1[j,2 * k + 1])
                dz1[j,2 * i + 1] += k * (np.sin(theta1[i][k]) * z1[j,2 * k] + np.cos(theta1[i][k]) * z1[j,2 * k + 1])
    return dz1

def cpg_output(z0, t0, tf):
    t_eval = np.arange(t0, tf, dt)
    sol = odeint(hopf6_3, z0, t_eval)


    return sol[-1, :]


time_start = time.time()

def two_layer_out(z0,z1,t0,tf):
    t_eval = np.arange(t0, tf, dt)
    sol = odeint(hopf6_3, z0, t_eval)
    count = 0
    zz_out=z1
    zz_out=np.resize(zz_out,(6,6,1))
    for t in np.arange(t0, tf-dt, dt):
        dz1 = sec_hopf(z1)
        count += 1
        if t == tf:
            break

        for j in range(6):
            z1[j,0:2]=sol[count,j*2:2*j+2]
        z1=z1 +dz1*dt
        zz_out=np.append(zz_out,np.resize(z1,(6,6,1)),axis=2)

    return zz_out[:,:,:]





def cpg_begin(z0):
    z0 = np.ones(12) * 0.5
    current_T = 0
    cpg_o = np.ones([1, 12]) * 0.5
    while current_T < 10:

        z0 = cpg_output(z0, current_T, current_T + 2 * dt)
        current_T += 2 * dt
        z1 = np.resize(z0, (1, 12))
        cpg_o = np.append(cpg_o, z1, axis=0)
        if current_T > 5 and abs(z0[2]) < 0.001:
            print("find")
            break
    print(z0)
    return z0

cf_base = 0.75  # 右边的零位状态　左边增加负号
ft_base = -0.75
amp_bc = 0.35
amp2 = 0.15

def cpg2theta(z):
    bc_theta = np.zeros(6)
    for j in range(6):
        bc_theta[j]=amp_bc*z[2*j]
    return bc_theta

def theta2cpg(bc_theta,z0):
    z=np.zeros(12)
    for j in range(6):
        z[2*j]=bc_theta[j]/amp_bc
        z[2*j+1]=z0[2*j+1]
    return z

def diff_cpg(z0,z1):
    dz=z1-z0
    if dz>0:
        dz=1
    else:
        dz=0
    return dz


z0 = np.ones(12) * 0.5
z1= np.ones((6,6)) * 0.5
cpg_o=two_layer_out(z0,z1,0,4)
t_eval = np.arange(0, 4, dt)
plt.figure(1)
for j in range(3):
    plt.subplot(3, 1, j + 1)
    plt.plot(t_eval, cpg_o[0, 2 * j,:])
plt.show()







start_t1=time.time()
z1=cpg_output(z0,0,10*dt)
print(z1)
end_t1=time.time()
last_t1=end_t1-start_t1
print(last_t1)





z0 = np.ones(12) * 0.5
z1 = cpg_begin(z0)
print(z1)
bc_theta=cpg2theta(z1)
print(bc_theta)
bc_theta+=0.2
z=theta2cpg(bc_theta,z1)
print(z)





time_end = time.time()
print(cpg_o[-1])
print("time ", time_end - time_start)
plt.figure(figsize=(12, 4))


t1=np.arange(0, 10+4*dt, 2*dt)
for j in range(6):
    plt.subplot(2, 3, j + 1)
    plt.plot(t1, cpg_o[:, 2 * j])
plt.show()
