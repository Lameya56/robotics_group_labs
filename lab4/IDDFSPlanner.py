import numpy as np
import time

class IDDFSPlanner(object):
    def __init__(self, planning_env):
        self.env = planning_env
        self.visited = np.zeros(self.planning_env.map.shape)

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

