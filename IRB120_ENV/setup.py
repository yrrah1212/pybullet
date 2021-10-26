# Credit to: https://gerardmaggiolino.medium.com/creating-openai-gym-environments-with-pybullet-part-2-a1441b9a4d8e
# Also used: https://github.com/openai/gym/blob/master/docs/creating_environments.md

from setuptools import setup

setup(
    name="irb120_env",
    version="0.0.1",
    install_requires=['gym', 'pybullet', 'numpy']
)