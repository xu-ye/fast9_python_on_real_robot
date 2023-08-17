import gym
import gym_hexapod
from stable_baselines.common.env_checker import check_env
from stable_baselines.common.callbacks import EvalCallback
from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines import DDPG
import pybullet as p
import pybullet_data as pd
import math
import numpy as np

from stable_baselines.ddpg.policies import FeedForwardPolicy


# Custom MLP policy of two layers of size 16 each
class CustomDDPGPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomDDPGPolicy, self).__init__(*args, **kwargs,
                                           layers=[64, 64],
                                           layer_norm=False,
                                           feature_extraction="mlp")






if __name__ == "__main__":

    env = gym.make('hexa-v0')

    check_env(env)
    n_actions = env.action_space.shape[-1]
    param_noise = None
    action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

    eval_callback = EvalCallback(env, best_model_save_path='./logs/',
                                 log_path='./logs/', eval_freq=50,
                                 deterministic=True, render=False)

    model = DDPG(CustomDDPGPolicy, env, verbose=1, param_noise=param_noise, action_noise=action_noise)
    model.learn(total_timesteps=240*10*30,callback=eval_callback)
    model.save("ddpg_hexapod3")

    del model  # remove to demonstrate saving and loading

    model = DDPG.load("ddpg_hexapod3")

    obs = env.reset()
    while True:
        action, _states = model.predict(obs)
        obs, rewards, dones, info = env.step(action)
        env.render()
        if dones:
            break

        print(f"state : {obs}, reward : {rewards}")



