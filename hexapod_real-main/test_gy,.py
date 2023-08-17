import gym
import Hexapod_Real
#from stable_baselines.common.env_checker import check_env
import pybullet as p
import pybullet_data as pd
import math
import numpy as np
#from stable_baselines import DDPG
#from stable_baselines import PPO1
#from stable_baselines import SAC

if __name__ == "__main__":

    env = gym.make('hexapod_real-v0')

    obs = env.reset()
 #   model = PPO1.load("./logs/PPO1/best_model.zip")

    while True:
        action = np.random.uniform(-0.2, 0.2, size=(6,1))+0.3
        #action, _states = model.predict(obs)
        state,share_obs,rewards,dones,info,available_actions = env.step(action)
        
        if dones[0]:
            break

        print(f"state : , reward : {rewards}",)