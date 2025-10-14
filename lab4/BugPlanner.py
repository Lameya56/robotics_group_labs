import numpy as np
import time

class BugPlanner:
    def __init__(self, planning_env, anystep=1, lookahead=1):
        self.env = planning_env
        self.anystep = anystep
        self.lookahead = lookahead

    def _get_next_wall_move(self, prev_pos, current_pos):
        # Helper function to find the next move when following a wall
        # TODO: Student Implementation
        pass

    def _bresenham(self, start, end):
        # M-line setup using Bresenham's algorithm
        # TODO: Student Implementation
        pass

    def Plan(self, start, goal):
        """
        Students should implement the bug 2 algorithm
        @param start: start configuration
        @param goal: goal configuration
        @return: a path of configurations
        """
        plan_time = time.time()
        # TODO: YOUR IMPLEMENTATION HERE
        path = [start]
        cost = 0

        plan_time = time.time() - plan_time
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)
        
        return np.array(path).T