import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from stable_baselines import DQN
from train_agent import BPEnv

env = BPEnv()

model = DQN.load("model.pkl")


env.testing = True
model.set_env(env)
obs = env.reset()
done = False
while not done:
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)



