import gym
import gym_hexapod
from stable_baselines.common.env_checker import check_env
from stable_baselines.common.policies import FeedForwardPolicy, register_policy
from stable_baselines.common.vec_env import DummyVecEnv
import pybullet as p
import pybullet_data as pd
import math
import numpy as np
from stable_baselines import DDPG
from stable_baselines.common.callbacks import EvalCallback

from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO1

class CustomPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs,
                                           net_arch=[dict(pi=[64],
                                                          vf=[64])],
                                           feature_extraction="mlp")

env = gym.make('hexa-v0')

eval_callback = EvalCallback(env, best_model_save_path='./logs/PPO1/',
                                 log_path='./logs/PPO1/', eval_freq=500,
                                 deterministic=True, render=False)

model = PPO1(MlpPolicy, env, verbose=1)
model.learn(total_timesteps=240*15*100,callback=eval_callback)
model.save("ppo1_cartpole4")

del model # remove to demonstrate saving and loading

model = PPO1.load("ppo1_cartpole4")

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    if dones:
        break
    env.render()
