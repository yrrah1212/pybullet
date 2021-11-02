from gym.envs.registration import register

register(
    id='IRB120-simple-v0', 
    entry_point='irb120_env.envs:IRB120ENV'
)