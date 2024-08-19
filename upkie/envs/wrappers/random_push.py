from gymnasium import Wrapper
import numpy as np


class RandomPush(Wrapper): 
    def __init__(self,env, push_prob = 0.1, push_amplitude = 10) :
        super().__init__(env)
        self.env = env
        self.push_prob =push_prob
        self.push_amplitude = push_amplitude
        self.env_get_spine_action = self.env.get_spine_action
        self.env.get_spine_action  = self.get_spine_action
    def get_spine_action(self ,action) :
        spine_action = self.env_get_spine_action(action)
        if np.random.binomial(1,self.push_prob):
            force = np.random.uniform(low = -self.push_amplitude, high = self.push_amplitude , size = 3)
            spine_action["bullet"] = {'external_forces':{"torso":{"force":force}}}
        else :
            spine_action["bullet"] = {'external_forces':{"torso":{"force":np.zeros(3)}}}
        return spine_action