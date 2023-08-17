# python
from gym.envs.registration import register

register(
        id='hexa-v0',
        entry_point='gym_hexapod.envs:HexaEnv',
)