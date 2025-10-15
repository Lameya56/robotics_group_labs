import numpy as np
from RRTTree import RRTTree
import time
import heapq

class PRMPlanner(object):
    def __init__(self, planning_env, num_samples, num_neighbors):
        self.env = planning_env
        self.tree = RRTTree(self.env)
        self.num_samples = num_samples
        self.num_neighbors = num_neighbors

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        # TODO: YOUR IMPLEMENTATION HERE
        # ---------------------------
        # 1. ROADMAP CONSTRUCTION
        # ---------------------------
        samples = [start_config]
        for _ in range(self.num_samples):
            s = self.env.sample()
            if self.env.state_validity_checker(s):
                samples.append(s)
        samples.append(goal_config)

        n = len(samples)
        adj_list = {i: [] for i in range(n)}  # adjacency list for the roadmap

        for i in range(n):
            dists = []
            for j in range(n):
                if i == j:
                    continue
                dist = self.env.compute_distance(samples[i], samples[j])
                dists.append((dist, j))
            # Take nearest neighbors
            dists.sort(key=lambda x: x[0])
            for _, j in dists[:self.num_neighbors]:
                if self.env.edge_validity_checker(samples[i], samples[j]):
                    cost = self.env.compute_distance(samples[i], samples[j])
                    adj_list[i].append((j, cost))
                    adj_list[j].append((i, cost))

        # ---------------------------
        # 2. PATH SEARCH (Dijkstra)
        # ---------------------------
        start_idx = 0
        goal_idx = len(samples) - 1

        costs = [float('inf')] * n
        parents = [None] * n
        costs[start_idx] = 0
        pq = [(0, start_idx)]
        visited = set()

        while pq:
            cost, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)

            if u == goal_idx:
                break

            for v, edge_cost in adj_list[u]:
                new_cost = cost + edge_cost
                if new_cost < costs[v]:
                    costs[v] = new_cost
                    parents[v] = u
                    heapq.heappush(pq, (new_cost, v))

        # ---------------------------
        # 3. RECONSTRUCT PATH
        # ---------------------------
        
        path = []
        node = goal_idx
        while node is not None:
            path.append(samples[node])
            node = parents[node]

        path.reverse()

        plan_time = time.time() - plan_time
        state_count = len(visited)
        total_cost = costs[goal_idx] if costs[goal_idx] < float('inf') else 0

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % total_cost)
        print("Planning Time: %ss" % plan_time)

        if parents[goal_idx] is None:
            print("No path found!")

        return np.concatenate(path, axis=1)
    


