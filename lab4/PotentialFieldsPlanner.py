import numpy as np
import time

class PotentialFieldsPlanner(object):
    def __init__(self, planning_env):
        self.env = planning_env

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        # TODO: YOUR IMPLEMENTATION HERE
        plan = [start_config, goal_config]
        
        cost = 0
        
        plan_time = time.time() - plan_time

        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)

        return np.array(plan).T

