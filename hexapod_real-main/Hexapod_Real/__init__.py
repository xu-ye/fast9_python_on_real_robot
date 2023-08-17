# python
from gym.envs.registration import register

register(
        id='hexapod_real-v0',
        entry_point='Hexapod_Real.envs:HexapodRealEnv',
)