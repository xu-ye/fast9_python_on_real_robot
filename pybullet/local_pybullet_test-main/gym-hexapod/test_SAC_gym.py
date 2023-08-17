import gym
import gym_hexapod
from stable_baselines.common.env_checker import check_env
from stable_baselines.sac.policies import MlpPolicy
from stable_baselines import SAC
from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines import DDPG
from stable_baselines.sac.policies import MlpPolicy
from stable_baselines import SAC

from stable_baselines.sac.policies import FeedForwardPolicy
from stable_baselines.common.vec_env import DummyVecEnv

import pybullet as p
import pybullet_data as pd
import math
import numpy as np
from stable_baselines.common.callbacks import EvalCallback


class CustomSACPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomSACPolicy, self).__init__(*args, **kwargs,
                                           layers=[64, 256,128, 64],
                                           layer_norm=False,
                                           feature_extraction="mlp")


if __name__ == "__main__":

    env = gym.make('hexa-v0')
    env = DummyVecEnv([lambda: env])

    eval_callback = EvalCallback(env, best_model_save_path='./logs/SAC/',
                                 log_path='./logs/SAC/', eval_freq=50,
                                 deterministic=True, render=False)

    model = SAC(MlpPolicy, env, verbose=1)
    model.learn(total_timesteps=240*10*100, log_interval=10,callback=eval_callback)
    model.save("sac_pendulum2")

    del model  # remove to demonstrate saving and loading

    model = SAC.load("sac_pendulum2")

    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()
        if dones:
            break

        print(f"state : {obs}, reward : {rewards}")
