import numpy as np
from RRTTree import RRTTree
import time

class PRMPlanner(object):
    def __init__(self, planning_env, num_samples, num_neighbors):
        self.env = planning_env
        self.tree = RRTTree(self.env)
        self.num_samples = num_samples
        self.num_neighbors = num_neighbors

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        # TODO: YOUR IMPLEMENTATION HERE
        plan = [start_config, goal_config]

        state_count = 0
        cost = 0

        plan_time = time.time() - plan_time

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)
        
        return np.array(plan).T

