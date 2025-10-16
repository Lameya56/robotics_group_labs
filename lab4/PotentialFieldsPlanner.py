import numpy as np
import time
from collections import deque

class PotentialFieldsPlanner(object):
    def __init__(self, planning_env):
        self.env = planning_env

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        plan = [start_config]
        current_pos = start_config.copy()
        max_iters = 10000
        step_size = 0.5
        rep_radius = 3  # obstacle influence radius

        for _ in range(max_iters):
            # Check if goal reached
            if self.env.goal_criterion(current_pos, goal_config):
                plan.append(goal_config)
                break

            # Compute repulsive force from nearby obstacles
            rep_force = np.zeros((2,1))
            x0, y0 = int(round(current_pos[0,0])), int(round(current_pos[1,0]))

            for dx in range(-rep_radius, rep_radius+1):
                for dy in range(-rep_radius, rep_radius+1):
                    x, y = x0+dx, y0+dy
                    if 0 <= x < self.env.map.shape[0] and 0 <= y < self.env.map.shape[1]:
                        if self.env.map[x, y] == 1:
                            dist = np.sqrt(dx**2 + dy**2)
                            if dist > 0:
                                rep_force += np.array([[(x0 - x)/dist**2], [(y0 - y)/dist**2]])

            # Attractive force towards goal
            goal_force = goal_config - current_pos

            # Combine forces
            net_force = rep_force + goal_force

            # Compute next position
            next_pos = current_pos + np.sign(net_force) * step_size

            if self.env.state_validity_checker(next_pos):
                plan.append(next_pos)
                current_pos = next_pos
            else:
                print("Local minimum detected. Using wavefront to escape...")
                escape_path = self._wavefront_escape(current_pos)
                if escape_path is None:
                    print("No valid escape path found — stopping.")
                    break
                for p in escape_path[1:]:  # skip current position
                    plan.append(p)
                current_pos = escape_path[-1]

        plan_time = time.time() - plan_time

        # Compute path cost
        cost = sum(self.env.compute_distance(plan[i], plan[i+1]) for i in range(len(plan)-1))
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)

        return np.hstack(plan)

    def _wavefront_escape(self, start_pos):
        """ Use BFS (wavefront) to find a nearby free cell that is farther from obstacles. """
        q = deque()
        visited = set()
        start = (int(round(start_pos[0,0])), int(round(start_pos[1,0])))
        q.append((start, [start]))
        visited.add(start)
        dirs = [(-1,0), (1,0), (0,-1), (0,1)]  # 4-connected grid

        while q:
            (x, y), path = q.popleft()

            # Escape criterion: find an open area away from obstacles
            if self._free_space_around(x, y, radius=2):
                return [np.array([[px],[py]]) for px,py in path]

            for dx, dy in dirs:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.env.map.shape[0] and
                    0 <= ny < self.env.map.shape[1] and
                    (nx, ny) not in visited and
                    self.env.map[nx, ny] == 0):
                    visited.add((nx, ny))
                    q.append(((nx, ny), path + [(nx, ny)]))
        return None

    def _free_space_around(self, x, y, radius=2):
        """ Returns True if the cell is at least `radius` distance away from obstacles. """
        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.env.map.shape[0] and 0 <= ny < self.env.map.shape[1]:
                    if self.env.map[nx, ny] == 1:
                        return False
        return True
