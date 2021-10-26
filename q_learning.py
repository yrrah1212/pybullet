import gym
import irb120_env
import pathlib

# TODO Need to implement an agent

def main():
    env = gym.make("IRB120-v0")
    rs = []
    for i in range(10):
        observation = env.reset()
        r = 0
        for t in range(10000):
            action = env.action_space.sample()
            # print(env.prev_error)
            observation, reward, done, info = env.step(action)
            if done:
                print(f"Feached the target in {t} steps")
                break
            r += reward

        # print(f"Goal: {env.goal}")
        rs.append(r)
    env.close()
    print(rs)


if __name__ == "__main__":
    main()