import numpy as np
import matplotlib.pyplot as plt

def generate_triangle(w,b,dt):
    T=2*np.pi/w
    t_eval = np.arange(0.0, T, dt)
    num_all=len(t_eval)
    num_up=int(b*T/dt)
    t_up=t_eval[0:num_up+1]
    t_down=t_eval[num_up+1:]
    f1=t_up*(2/t_up[num_up])-1
    f1_1=np.ones(num_up)
    f2=1-(t_down-t_down[0]+1*dt)*(2/(T-t_up[num_up]-dt))
    f2_1=np.ones(num_all-num_up)
    #f=f1.append(f2)
    f=np.append(f1,f2)
    f=np.tanh(f*2)
    f_1=np.append(f1_1,f2_1)
    return f,t_eval,f_1

def shift_phase(y,w,dt,phase):
    T=2*np.pi/w
    shift_num=int(T*phase/dt)
    y1=y[shift_num:]
    y2=y[0:shift_num]
    result=np.append(y1,y2)
    return result


def output_cpg(phase_all,w,b,dt):
    # phase_all  :躯干层相对于0腿的相位差 肢体层的相位差
    y0,t0,y0_1=generate_triangle(w,b,dt)
    length=len(y0)
    offset=0.85
    offset=0
    ini_index=np.argwhere(abs(y0-offset)<5e-3)
    #per=(ini_index[1]-ini_index[0])/length
    y01=y0[ini_index[0][0]:]
    y02=y0[0:ini_index[0][0]]
    y0_1=np.append(y01,y02)
    
    zzn=np.zeros((6,6,length))
    for i in range(6):
        zzn[i,0,:]=shift_phase(y0,w,dt,phase_all[i])
    
        
        zzn[i,2,:]=shift_phase(y0_1,w,dt,phase_all[i])
        zzn[i,4,:]=shift_phase(y0_1,w,dt,phase_all[i])
    return zzn

def cpg2theta_reshape( zz):
        theta = np.zeros((6, 3))
        bound0=0.85
        bound0=0
        amp_bc = 0.3
        amp_ft = 0.188
        #amp_bc = 0.3
        #amp_ft = 0.188
        old_z = np.zeros(12)
        fre = 0
        theta1 = [[0, 0, 0], [1 / 4, 0, 0], [1 / 4, 0, 0]]
        #cf_base = 0.35  # 右边的零位状态　左边增加负号
        #ft_base = -0.35
        cf_base = 0.3  # 右边的零位状态　左边增加负号
        ft_base = -0.33
        old_zz = np.zeros((6,6))
        

        for j in range(6):
            theta[j, 0] = amp_bc * zz[j, 0]
            if j >=3:
                theta[j, 0] = -amp_bc * zz[j, 0]
            if j < 3:
                if zz[j, 2] <bound0:
                    theta[j, 1] = -cf_base + amp_ft * 0
                else:
                    theta[j, 1] = -cf_base + amp_ft * (zz[j, 2]-bound0)*1/(1-bound0) # 伸展为加
                if zz[j, 4]<bound0:
                    theta[j, 2] = -ft_base + amp_ft * 0
                else:
                    theta[j, 2] = -ft_base + amp_ft * (zz[j, 4]-bound0)*1/(1-bound0)

            else:
                if zz[j, 2] <bound0:
                    theta[j, 1] = cf_base - amp_ft * 0  # cf
                else:
                    theta[j, 1] = cf_base - amp_ft * (zz[j, 2]-bound0)*1/(1-bound0)  # cf
                if zz[j, 4]<bound0:
                    theta[j, 2] = ft_base - amp_ft * 0  # ft
                else:
                    theta[j, 2] = ft_base - amp_ft * (zz[j, 4]-bound0)*1/(1-bound0)  # ft
            if j==2 or j==1:
                if zz[j, 2] <bound0:
                    theta[j, 1] = -cf_base + amp_ft * 0+0.002
                else:
                    theta[j, 1] = -cf_base + amp_ft * (zz[j, 2]-bound0)*1/(1-bound0) # 伸展为加
                if zz[j, 4]<bound0:
                    theta[j, 2] = -ft_base + amp_ft * 0-0.002
                else:
                    theta[j, 2] = -ft_base + amp_ft * (zz[j, 4]-bound0)*1/(1-bound0)



        return theta

'''
dt=1/240
w=1*np.pi
b=1/6
phase_all=[0/6,1/6,2/6,3/6,4/6,5/6,1/4,1/4]
y,x,y1=generate_triangle(1*np.pi,1/6,1/240)
y2=shift_phase(y,1*np.pi,dt,1/4)
plt.figure()
plt.plot(x,y,x,y2)
plt.show()
zzn1=output_cpg(phase_all,w,b,dt)
tf=10
N=int(tf/(2*np.pi/w))
zzn=np.tile(zzn1,N)
t_eval = np.arange(0.0, tf, dt)
for j in range(6):
    plt.subplot(3, 2, j+1)
    plt.plot(t_eval, zzn[j,0,:])
plt.show()
for j in range(3):
    plt.subplot(3, 1, j+1)
    plt.plot(t_eval, zzn[2,2*j,:])
plt.show()
plt.show()
for j in range(3):
    plt.subplot(3, 1, j+1)
    plt.plot(t_eval, zzn[1,2*j,:])
plt.show()
'''

