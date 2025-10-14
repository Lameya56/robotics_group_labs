import numpy as np
import time

class VisibilityGraphPlanner(object):
    def __init__(self, planning_env):
        self.env = planning_env

    def _get_obstacle_vertices(self):
        """
        A heuristic to extract obstacle corners from a grid map. It identifies
        2x2 blocks that represent a corner and adds the free-space cells
        from that block as vertices.
        """
        map_ = self.planning_env.map
        vertices = set()
        # Iterate over all possible 2x2 block top-left corners
        for r in range(map_.shape[0] - 1):
            for c in range(map_.shape[1] - 1):
                block = map_[r:r+2, c:c+2]
                # A corner is where free space meets obstacle space.
                # Sum of 1 or 3 indicates a corner in the 2x2 block.
                if np.sum(block) == 1 or np.sum(block) == 3:
                    # Find the free-space cell(s) in this block
                    for dr, dc in [(0, 0), (0, 1), (1, 0), (1, 1)]:
                        check_r, check_c = r + dr, c + dc
                        if map_[check_r, check_c] == 0:
                            # Add the coordinates of the valid, free-space cell
                            vertices.add((check_r, check_c))
        
        # Convert the set of tuple coordinates to the required list of numpy arrays
        return [np.array(v).reshape(2, 1) for v in vertices]

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        obs_vertices = self.get_obstacle_vertices()
        # TODO: YOUR IMPLEMENTATION HERE
        plan = [start_config, goal_config]

        state_count = 0
        cost = 0
        
        plan_time = time.time() - plan_time

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)

        return np.array(plan).T

