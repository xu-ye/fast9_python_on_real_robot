from actuator_model2 import *
#actuator_model_error(0.9582,0.03821)
#actuator_model_error(0.01621,0.3618,8.014)
for i in range(20):
    #0.01621 0.3618 
    #D_gain=0.39+0.0005*i
    P_gain=0.0164+0.00001*i
    #print(" D_gain",P_gain)
    #actuator_model_error(0.1647,0.397,8.92)
#actuator_model_error(0.01647,0.397,8.92)
#actuator_control_pd__multi(0.01647,0.397,8.92)

#actuator_control_pd_force_multi(5.843,0.1316,0.0203)
#actuator_control_pd_force_2(5.843,0.1316,0.0203)

#actuator_control_pd_force_multi_stable(5.843,0.1316,0.0203)
#actuator_control_pd_force_multi_stable(9,0.0001,0.032)

#actuator_control_pd_force_multi_support_swing(27,0.531,0.0203)

#actuator_control_pd_force_multi_obstacle(27,0.531,0.0203)
#actuator_control_pd_force_multi_obstacle(60,0.531,0.0203)

#actuator_control_pd__multi_obstacle(0.01647,0.397,8.92)
actuator_control_pd_10ms(0.01647,0.397,8.92)



