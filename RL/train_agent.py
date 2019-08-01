import sys
import os

sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import gym
from gym.utils import seeding
from z3 import *
import socket
import time
import math
import subprocess
from gym.spaces.discrete import Discrete
from gym import spaces

global roverSocket, half_speed_forward, full_speed_forward, half_speed_backwards, full_speed_backwards, need_to_spin
exception = False


def SplitInData(s):
    s1 = s.split(';')
    s2 = s1[0].split(',')
    return s2


def getTelem():
    global RoverPx, RoverPy, Compass, LeaderPx, LeaderPy, Dist, DDeg, GDDeg, exception
    time.sleep(0.03)
    print("started telem")
    roverSocket.sendall(b"player2,GPS()\n")
    try:
        rData = repr(roverSocket.recv(1024))
    except Exception as e:
        # exception = True
        return False
    RoverGPSData = SplitInData(rData)
    RoverPx = float(RoverGPSData[1])
    RoverPy = float(RoverGPSData[2])
    print("Rover Pos x:", RoverPx)
    print("Rover Pos y:", RoverPy)

    roverSocket.sendall(b"player2,getCompass()\n")
    rData = repr(roverSocket.recv(1024))
    RoverCompassData = SplitInData(rData)
    Compass = float(RoverCompassData[1])
    print("Compass:", Compass)

    roverSocket.sendall(b"ball,GPS()\n")
    lData = repr(roverSocket.recv(1024))
    LeaderGPSData = SplitInData(lData)
    LeaderPx = float(LeaderGPSData[1])
    LeaderPy = float(LeaderGPSData[2])
    print("Leader Pos x:", LeaderPx)
    print("Leader Pos y:", LeaderPy)

    LeaderDistanceData = pow(
        ((RoverPx - LeaderPx) * (RoverPx - LeaderPx) + (RoverPy - LeaderPy) * (RoverPy - LeaderPy)), (1 / 2))
    Dist = LeaderDistanceData
    print("Distance:", Dist)

    LRDeg = math.atan2((LeaderPx - RoverPx), -(LeaderPy - RoverPy))
    LRDeg = (LRDeg / math.pi) * 180
    DDeg = (90 - Compass) - LRDeg

    if (abs(DDeg) >= 360):
        if (DDeg > 0):
            DDeg = DDeg - 360;
        else:
            DDeg = DDeg + 360;

    if (abs(DDeg) > 180):
        if (DDeg > 180):
            DDeg = DDeg - 360

        if (DDeg < (-180)):
            DDeg = DDeg + 360
    print("DDeg:", DDeg)
    print("#################################")

    GLRDeg = math.atan2((-50 - RoverPx), -(0 - RoverPy))
    GLRDeg = (GLRDeg / math.pi) * 180
    GDDeg = (90 - Compass) - GLRDeg

    if (abs(GDDeg) >= 360):
        if (GDDeg > 0):
            GDDeg = GDDeg - 360;
        else:
            GDDeg = GDDeg + 360;

    if (abs(GDDeg) > 180):
        if (GDDeg > 180):
            GDDeg = GDDeg - 360

        if (GDDeg < (-180)):
            GDDeg = GDDeg + 360
    print("GDDeg:", GDDeg)
    print("#################################")
    return true


# B-Threads
global TooFar, TooClose, MAX_PWR, MAX_SPIN
global true, false, BInRobot, Su, reward, m

true = BoolSort().cast(True)
false = BoolSort().cast(False)
m = None

BInRobot = Bool('BInRobot')
Su = Int('Su')
reward = Real('reward')
spin = Real('spin')
forward = Int('forward')

TooFar = 5
TooClose = 3.5
MAX_PWR = 100
MAX_SPIN = 100

class Request:
    def __init__(self, variables=None):
        self.variables = variables


class WaitFor:
    pass


class Block:
    pass


class BPEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.action_space = Discrete(8)
        # self.observation_space = spaces.Box(0, 106, shape=(1,), dtype=np.float32)
        low = np.array([0, 0])
        high = np.array([360, 112])
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.start_time = None
        self.testing = True
        self.first_time = None
        self.tickets = None
        self.connected = None
        self.scenarios = None
        self.simulation_path = None

    def step(self, action):
        global exception
        self.update_action_variables(action)
        interrupted = False
        global m  # A variable where the solved model is published

        if self.first_time:
            self.first_time = False
            self.tickets = []  # A variable containing the tickets issued by the scenarios
            self.tickets = [{'bt': bt} for bt in self.scenarios]
            # Run all scenario objects to their initial yield
            for l in self.tickets:
                bt = l['bt']
                l.clear()
                ll = bt.send(m)
                l.update(ll)
                l.update({'bt': bt})
        else:
            # Reset the list of tickets before rebuilding it
            oldtickets = self.tickets
            self.tickets = []
            # Run the scenarios to their next yield and collect new tickets
            for oldticket in oldtickets:
                # Check if the scenario waited for the computed assignment
                if WaitFor in oldticket and is_true(m.eval(oldticket[WaitFor])):

                    bt = oldticket['bt']
                    oldticket.clear()
                    ll = bt.send(m)
                    oldticket.update(ll)
                    oldticket.update({'bt': bt})
                    self.tickets.append(oldticket)

                else:
                    # Copy the old tickets to the new list
                    self.tickets.append(oldticket)

        (request, block) = (False, False)

        no_request = True
        for ticket in self.tickets:
            if Request in ticket:
                request = Or(request, ticket[Request])
                no_request = false
            if Block in ticket:
                block = Or(block, ticket[Block])

        if no_request:
            request = True
        # Compute a satisfying assignment and break if it does not exist
        sl = Solver()
        sl.add(And(request, Not(block)))
        if sl.check() == sat:
            m = sl.model()
        else:
            interrupted = True

        cur_reward = float(m[reward].as_decimal(3))
        if not time.time() - self.start_time < 220:
            interrupted = True
            self.close()
        print("reward", cur_reward)
        return np.array([DDeg, Dist]), cur_reward, interrupted, {}

    def reset(self):
        global roverSocket, m
        self.first_time = True
        m = None
        self.scenarios = [
            invariants(),
            get_ball_reward(),
            move_towards_ball(),
            spin_to_ball(),
            spin_to_goal(),
            get_ball(),
            shoot_ball(),
            logger(),
            telem_updater()
        ]
        self.start_time = time.time()
        print(self.start_time)
        if not self.testing:
            subprocess.call(["open", self.simulation_path])
            time.sleep(6)
        roverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        roverSocket.settimeout(1.0)
        roverSocket.connect(('127.0.0.1', 9003))
        roverSocket.settimeout(1.0)

        print("after connect")
        self.connected = True
        getTelem()
        time.sleep(0.1)

        return np.array([DDeg, Dist])

    def render(self, mode='human', close=False):
        raise NotImplementedError

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def close(self):
        roverSocket.close()
        self.close_simulation()
        self.connected = False
        print("ENDED")

    def close_simulation(self):
        if not self.testing:
            pr = subprocess.check_output("pgrep mac", shell=True)
            for p in pr.decode().split("\n")[:-1]:
                subprocess.call(["kill", "-9", p])
        print("ENDED")

    def update_action_variables(self, action):
        global half_speed_forward, full_speed_forward, half_speed_backwards, full_speed_backwards, need_to_spin
        half_speed_forward, full_speed_forward, half_speed_backwards, full_speed_backwards, need_to_spin = (
        action in [0, 4], action in [1, 5], action in [2, 6], action in [3, 7], action in [0, 1, 2, 3])


def invariants():
    yield {Block: Not(And(forward >= -MAX_PWR,
                            forward <= MAX_PWR,
                            Or(spin == 0, spin == MAX_SPIN, spin == -MAX_SPIN),
                            Or(BInRobot == False, BInRobot == True))),
           WaitFor: false}


def get_ball_reward():
    m = yield {Block: reward != -0.05, WaitFor: true}
    while True:
        if is_true(m[BInRobot]):
            m = yield {Block: reward != 1, WaitFor: true}
        else:
            m = yield {Block: reward != -0.05, WaitFor: true}


def move_towards_ball():
    m = yield {WaitFor: true}
    while True:
        if is_false(m[BInRobot]):
            if half_speed_forward:
                m = yield {Request: forward == MAX_PWR / 2, WaitFor: true}
            if full_speed_forward:
                m = yield {Request: forward == MAX_PWR, WaitFor: true}
            if half_speed_backwards:
                m = yield {Request: forward == -MAX_PWR / 2, WaitFor: true}
            if full_speed_backwards:
                m = yield {Request: forward == -MAX_PWR, WaitFor: true}
        else:
            m = yield {WaitFor: true}


def spin_to_ball():
    m = yield {WaitFor: true}
    while True:
        if is_false(m[BInRobot]):
            ang = angle_between_robot_and_ball(m)
            if need_to_spin and ang > 0:
                m = yield {Request: spin > 0, Block: spin <= 0, WaitFor: true}
            elif need_to_spin and ang < 0:
                m = yield {Request: spin < 0, Block: spin >= 0, WaitFor: true}
            else:
                m = yield {Block: spin != 0, WaitFor: true}
        else:
            m = yield {WaitFor: true}


def spin_to_goal():
    m = yield {WaitFor: true}
    while True:
        if is_true(m[BInRobot]):
            if (abs(GDDeg) > 8):
                if (GDDeg > 0):
                    m = yield {Request: spin > 0, Block: spin <= 0, WaitFor: true}
                else:
                    m = yield {Request: spin < 0, Block: spin >= 0, WaitFor: true}
            else:
                m = yield {WaitFor: true}
        else:
            m = yield {WaitFor: true}


def get_ball():
    m = yield {WaitFor: true}
    while True:
        if is_false(m[BInRobot]):
            if Dist < 4.3:
                m = yield {Block: Not(And(Su == -100, BInRobot == True)),
                           WaitFor: true}
            else:
                m = yield {Block: Not(And(Su == -100, BInRobot == False)),
                           WaitFor: true}
        else:
            m = yield {WaitFor: true}


def shoot_ball():
    m = yield {Block: BInRobot != False, WaitFor: true}
    while True:
        if is_true(m[BInRobot]):
            if abs(GDDeg) < 8:
                m = yield {Block: Not(And(Su == 100, BInRobot == False)),
                           WaitFor: true}
            else:
                m = yield {Block: Not(And(Su == -100, BInRobot == True)),
                           WaitFor: true}
        else:
            m = yield {WaitFor: true}


def telem_updater():
    while True:
        getTelem()
        yield {WaitFor: true}


def logger():
    while True:
        m = yield {WaitFor: true}
        spin_speed = m[spin]
        Suction = m[Su]
        forward_speed = m[forward]
        if spin_speed == 0:
            stringout = "player2,moveForward(" + str(forward_speed) + ")\n"
            roverSocket.sendall(stringout.encode('utf-8'))
            print(stringout)
        if (is_false(m[BInRobot])):
            if spin_speed.as_long() > 0:
                stringout = "player2,moveForward(0)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
                time.sleep(0.02)
                stringout = "player2,spin(100)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
                time.sleep(0.02)
                stringout = "player2,spin(0)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
            if spin_speed.as_long() < 0:
                stringout = "player2,moveForward(0)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
                time.sleep(0.02)
                stringout = "player2,spin(-100)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
                time.sleep(0.02)
                stringout = "player2,spin(0)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
        else:
            if spin_speed.as_long() > 0:
                stringout = "player2,moveForward(0)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
                time.sleep(0.02)
                stringout = "player2,spin(100)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
                time.sleep(0.02)
                stringout = "player2,spin(0)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
            if spin_speed.as_long() < 0:
                stringout = "player2,moveForward(0)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
                time.sleep(0.02)
                stringout = "player2,spin(-100)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)
                time.sleep(0.02)
                stringout = "player2,spin(0)\n"
                roverSocket.sendall(stringout.encode('utf-8'))
                print(stringout)

        stringout = "player2,setSuction(" + str(Suction) + ")\n"
        roverSocket.sendall(stringout.encode('utf-8'))
        print(stringout)


def angle_between_robot_and_ball(m):
    return DDeg


import os
import sys

import gym
import numpy as np
import matplotlib.pyplot as plt

from stable_baselines.bench import Monitor
from stable_baselines.results_plotter import load_results, ts2xy
from stable_baselines.deepq.policies import MlpPolicy
from stable_baselines import DQN
from stable_baselines.deepq.policies import FeedForwardPolicy

import baselines.common.tf_util as U
import tensorflow as tf

best_mean_reward, n_steps = -np.inf, 0


def callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    global n_steps, best_mean_reward
    # Print stats every 1000 calls
    # print(n_steps)
    if (n_steps + 1) % 1000 == 0:
        # if (n_steps + 1) % 10 == 0:
        # Evaluate policy training performance
        x, y = ts2xy(load_results(log_dir), 'timesteps')
        if len(x) > 0:
            mean_reward = np.mean(y[-15:])
            print(x[-1], 'timesteps')
            print(
                "Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(best_mean_reward, mean_reward))
            # New best model, you could save the agent here
            if mean_reward > best_mean_reward:
                best_mean_reward = mean_reward
                # Example for saving best model
                print("Saving new best model")
                _locals['self'].save(log_dir + 'model.pkl')
    n_steps += 1
    return True


# Custom MLP policy of two layers of size 32 each
class CustomPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs,
                                           layers=[5],
                                           layer_norm=False,
                                           feature_extraction="mlp")


if __name__ == "__main__":
    log_dir = os.path.dirname(os.path.realpath(__file__)) + "/"
    os.makedirs(log_dir, exist_ok=True)
    # Create and wrap the environment
    env = BPEnv()
    env.testing = False
    env.simulation_path = "/Users/tomyaacov/Desktop/university/thesis/ChallengeProblem/mac/mac.app"
    env = Monitor(env, log_dir, allow_early_resets=True)

    model = DQN(CustomPolicy, env, verbose=1)

    model.learn(total_timesteps=400000, callback=callback)
    model.save(log_dir + 'model.pkl')

    if env.env.connected:
        env.env.close()
