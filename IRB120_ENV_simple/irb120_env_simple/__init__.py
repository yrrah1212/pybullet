from gym.envs.registration import register

register(
    id='IRB120-v0-simple', 
    entry_point='irb120_env.envs:IRB120ENV'
)