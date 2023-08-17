import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.policies import FeedForwardPolicy, register_policy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import TRPO

import pybullet as p
import pybullet_data as pd
import math
import numpy as np
import gym_hexapod


class CustomPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs,
                                           net_arch=[dict(pi=[64, 256,128, 64],
                                                          vf=[64, 256,128, 64])],
                                           feature_extraction="mlp")

env = gym.make('hexa-v0')
env = DummyVecEnv([lambda: env])


model = TRPO(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=240*8*1000, log_interval=10)
model.save("hexa-trpo1")




del model # remove to demonstrate saving and loading

model = TRPO.load("hexa-trpo1")

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()