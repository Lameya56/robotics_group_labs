import numpy as np
import time
import heapq

class AStarPlanner(object):    
    def __init__(self, planning_env, epsilon):
        self.env = planning_env
        self.nodes = {}
        self.epsilon = epsilon
        self.visited = np.zeros(self.env.map.shape)

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        # TODO: YOUR IMPLEMENTATION HERE
        start = tuple(start_config.flatten().astype(int))
        goal = tuple(goal_config.flatten().astype(int))
        # Priority queue for frontier (f, g, config)
        frontier = []
        heapq.heappush(frontier, (0,0, start))
        # g-calues and parent dictionary
        g_values = {start: 0}
        parents = {start:None}
        
        state_count = 0
        # A* main loop
        while frontier:
            _, g, current = heapq.heappop(frontier)

            # Skip if already visited
            if self.visited[current]:
                continue

            self.visited[current] = 1
            state_count += 1

            # Check goal condition
            current_config = np.array(current).reshape((2, 1))
            if self.env.goal_criterion(current_config, goal_config):
                print("Goal reached!")
                break

            # Explore neighbors (8-connected grid)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = (current[0] + dx, current[1] + dy)
                    neighbor_config = np.array(neighbor).reshape((2, 1))

                    # Skip invalid or out-of-bounds
                    if not self.env.state_validity_checker(neighbor_config):
                        continue

                    # Cost to move to neighbor
                    new_cost = g + self.env.compute_distance(current_config, neighbor_config)

                    # Only update if it's a better path
                    if neighbor not in g_values or new_cost < g_values[neighbor]:
                        g_values[neighbor] = new_cost
                        f = new_cost + self.epsilon * self.env.h(neighbor_config)
                        heapq.heappush(frontier, (f, new_cost, neighbor))
                        parents[neighbor] = current

        # Reconstruct path if goal found
        plan = []
        node = min(g_values.keys(), key=lambda n: self.env.compute_distance(np.array(n).reshape(2, 1), goal_config))
        while node is not None:
            plan.append(np.array(node).reshape((2, 1)))
            node = parents[node]
        plan.reverse()

        plan = np.concatenate(plan, axis=1)
        cost = g_values[min(g_values.keys(), key=lambda n: self.env.compute_distance(np.array(n).reshape(2, 1), goal_config))]

        plan_time = time.time() - plan_time

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)

        return plan
