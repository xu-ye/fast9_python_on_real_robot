import gym
import gym_hexapod
from stable_baselines.common.env_checker import check_env
import pybullet as p
import pybullet_data as pd
import math
import numpy as np
from stable_baselines import DDPG
from stable_baselines import PPO1
from stable_baselines import SAC

if __name__ == "__main__":

    env = gym.make('hexa-v0')

    obs = env.reset()
    model = PPO1.load("./logs/PPO1/best_model.zip")

    while True:
        action = np.random.uniform(-0.01, 0.01, size=(12,))
        #action, _states = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        if done:
            break

        print(f"state : {obs}, reward : {reward}")