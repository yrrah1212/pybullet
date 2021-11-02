from gym.envs.registration import register

register(
    id='IRB120-simple-v0', 
    entry_point='irb120_env_simple.envs:IRB120ENV_simple'
)