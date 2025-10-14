import numpy as np
import time

class AStarPlanner(object):    
    def __init__(self, planning_env, epsilon):
        self.env = planning_env
        self.nodes = {}
        self.epsilon = epsilon
        self.visited = np.zeros(self.env.map.shape)

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        # TODO: YOUR IMPLEMENTATION HERE

        plan = []
        plan.append(start_config)
        plan.append(goal_config)

        state_count = 0
        cost = 0

        plan_time = time.time() - plan_time

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)

        return np.concatenate(plan, axis=1)
