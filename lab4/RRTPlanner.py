import numpy as np
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, bias = 0.05, eta = 1.0, max_iter = 10000):
        self.env = planning_env         # Map Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                # Goal Bias
        self.max_iter = max_iter        # Max Iterations
        self.eta = eta                  # Distance to extend

    def Plan(self, start_config, goal_config):
        # TODO: YOUR IMPLEMENTATION HERE

        plan_time = time.time()

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        state_count = 0

        for i in range(self.max_iter):
            # Sample random configuration
            x_rand = self.sample(goal_config)

            # Find nearest vertex
            vid_near, x_near = self.tree.GetNearestVertex(x_rand)

            # Extend toward x_rand
            x_new = self.extend(x_near, x_rand)

            # Check edge validity
            if self.env.edge_validity_checker(x_near, x_new):
                new_id = self.tree.AddVertex(x_new)
                self.tree.AddEdge(vid_near, new_id)
                state_count += 1

                # Check if goal reached
                if self.env.goal_criterion(x_new, goal_config):
                    print("Goal reached!")
                    break

        # Reconstruct path
        plan = []
        last_id = len(self.tree.vertices) - 1
        while last_id != self.tree.GetRootID():
            v = self.tree.vertices[last_id]
            plan.append(v)
            last_id = self.tree.edges[last_id]
        plan.append(self.tree.vertices[0])
        plan.reverse()

        plan = np.concatenate(plan, axis=1)
        cost = self.compute_path_cost(plan)
        plan_time = time.time() - plan_time

        print("States Expanded:", state_count)
        print("Cost:", cost)
        print("Planning Time:", plan_time, "s")

        return plan


    def extend(self, x_near, x_rand):
        # TODO: YOUR IMPLEMENTATION HERE
        """Return a new configuration by stepping from x_near toward x_rand by eta distance."""
        direction = x_rand - x_near
        dist = np.linalg.norm(direction)
        if dist < self.eta:
            return x_rand
        direction = (direction / dist) * self.eta
        x_new = x_near + direction
        return x_new
        

    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()

    def compute_path_cost(self, plan):
        cost = 0
        for i in range(plan.shape[1] - 1):
            cost += self.env.compute_distance(plan[:, i].reshape(2, 1),
                                              plan[:, i+1].reshape(2, 1))
        return cost