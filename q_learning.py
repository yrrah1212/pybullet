import gym
import irb120_env
import pathlib


def main():
    env = gym.make("IRB120-v0")
    observation = env.reset()
    for t in range(5000):
        print(observation)
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if done:
            print(f"Feached the target in {t} steps")
            break

    env.close()


if __name__ == "__main__":
    main()