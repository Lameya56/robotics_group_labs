import numpy as np
import time

class IDDFSPlanner(object):
    def __init__(self, planning_env):
        self.env = planning_env
        self.visited = np.zeros(planning_env.map.shape)

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        # TODO: YOUR IMPLEMENTATION HERE
        plan = []
        max_depth = 1
        state_count = 0

        while not plan and state_count >= 0:
            state_count, plan = self.dfs(start_config, goal_config, max_depth)
            if self.checkState():
                print("Unable to figure out a path")
                break

            self.visited = np.zeros(self.env.map.shape)
            max_depth += 1
        
        cost = self.calculateCost(plan)
        plan_time = time.time() - plan_time

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)
        
        return np.hstack(plan)
    
    def dfs(self, start_config, goal_config, max_depth):
        state_count = 0 # Total # of coordinates we searched through        
        stack = [(start_config, None, 0)]
        parents = dict() # Current Config : Parent Config
        parents[tuple(start_config.flatten())] = None
        directions = [
            [0,1], # Up
            [0,-1], # Down
            [1,0], # Right
            [-1, 0], # Left
            [1,1], # Upper Right
            [-1,-1], # Lower Left
            [1,-1], # Lower Right
            [-1,1], # Upper Left
        ]

        while stack:
            current_location, parent_location, depth = stack.pop()

            x = int(current_location[0,0])
            y = int(current_location[1,0])  

            # If the location is an obstacle, invalid location, or a previously explored location
            if not self.env.state_validity_checker(current_location) or self.visited[x, y] == 1:
                continue

            state_count += 1      

            if self.env.goal_criterion(current_location, goal_config):
                print("Goal Reached!")
                plan = self.tracePlan(parents, current_location, goal_config)
                return state_count, plan
            
            self.visited[x,y] = 1

            if depth < max_depth:
                for dx, dy in directions:
                    next_x = x + dx
                    next_y = y + dy
                    child = np.array([[next_x], [next_y]])

                    if self.env.state_validity_checker(child) and self.visited[next_x, next_y] == 0:
                        stack.append((child, current_location, depth + 1))
                        parents[tuple(child.flatten())] = tuple(current_location.flatten())
            
        return state_count, []


    def tracePlan(self, parents, current_location, goal):
        plan = []

        current = tuple(current_location.flatten())

        while current:
            plan.append(np.array([[current[0]], [current[1]]]))
            current = parents[current]

        plan.reverse()
        return plan
    
    def calculateCost(self, plan):
        cost = 0
        for index in range(len(plan) - 1):
            cost += self.env.compute_distance(plan[index], plan[index + 1])
        return cost

    def checkState(self):
        for x in range(self.visited.shape[0]):
            for y in range(self.visited.shape[1]):
                if self.visited[x,y] == 0 and self.env.map[x,y] != 1:
                    return False
        
        return True
