import numpy as np
import time

class PotentialFieldsPlanner(object):
    def __init__(self, planning_env):
        self.env = planning_env
        # Tuning parameters for optimal performance
        self.k_att = 2.0    # Attractive gain for faster convergence
        self.k_rep = 10.0    # Repulsive gain for safety
        self.rho_0 = 10.0    # Obstacle influence radius

    def compute_potential_gradient(self, robot_position, goal_position):
        """
        Compute gradient of potential field P(x) at robot position
        Uses sensor information (environment map) to estimate ∇P(x)
        Returns motor vector u to follow gradient
        """
        # 1. ATTRACTIVE POTENTIAL GRADIENT (minimize travel time)
        # Quadratic potential near goal for stability, conic far away for speed
        dist_to_goal = np.linalg.norm(robot_position - goal_position)
        
        if dist_to_goal <= 2.0:  # Close to goal - quadratic for stability
            attractive_gradient = self.k_att * (robot_position - goal_position)
        else:  # Far from goal - conic for speed
            attractive_gradient = self.k_att * (robot_position - goal_position) / dist_to_goal
        
        # 2. REPULSIVE POTENTIAL GRADIENT (avoid crashes)
        repulsive_gradient = np.zeros((2, 1))
        x, y = int(robot_position[0,0]), int(robot_position[1,0])
        
        # Sensor reading: check obstacles within influence radius
        sensor_range = int(self.rho_0)
        for dx in range(-sensor_range, sensor_range + 1):
            for dy in range(-sensor_range, sensor_range + 1):
                nx, ny = x + dx, y + dy
                
                # Check if obstacle detected by sensor
                if (0 <= nx < self.env.map.shape[0] and 
                    0 <= ny < self.env.map.shape[1] and 
                    self.env.map[nx, ny] == 1):
                    
                    obstacle_pos = np.array([[nx], [ny]])
                    dist_to_obs = np.linalg.norm(robot_position - obstacle_pos)
                    
                    # Only consider obstacles within influence radius
                    if 0 < dist_to_obs <= self.rho_0:
                        # Repulsive potential: grows to infinity near obstacles
                        dir_to_robot = (robot_position - obstacle_pos) / dist_to_obs
                        repulsive_magnitude = self.k_rep * (1/dist_to_obs - 1/self.rho_0) * (1/dist_to_obs**2)
                        repulsive_gradient += repulsive_magnitude * dir_to_robot
        
        # 3. TOTAL POTENTIAL GRADIENT ∇P(x)
        total_gradient = attractive_gradient + repulsive_gradient
        
        # 4. MOTOR VECTOR u (follow negative gradient = downhill)
        motor_vector = -total_gradient
        
        return motor_vector, attractive_gradient, repulsive_gradient

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        
        plan = [start_config.copy()]
        current = start_config.copy()
        
        for iteration in range(1000):
            # Stability at goal: check if sufficiently close
            if np.linalg.norm(current - goal_config) < 0.5:
                plan.append(goal_config.copy())
                print(f"Goal reached stably in {iteration} iterations")
                break
            
            # Compute motor vector u from potential field gradient
            motor_vector, att_grad, rep_grad = self.compute_potential_gradient(current, goal_config)
            
            # Normalize for consistent movement (minimize travel time)
            motor_norm = np.linalg.norm(motor_vector)
            if motor_norm > 0:
                step_direction = motor_vector / motor_norm
            else:
                step_direction = np.zeros((2, 1))
            
            # Move in gradient direction
            next_pos = current + step_direction
            
            # Safety check: ensure we don't crash
            if self.env.state_validity_checker(next_pos):
                plan.append(next_pos.copy())
                current = next_pos.copy()
            else:
                # Emergency stop if movement would cause crash
                print("Emergency stop: potential crash detected")
                break
        
        # Calculate actual travel cost
        cost = 0
        for i in range(len(plan) - 1):
            cost += self.env.compute_distance(plan[i], plan[i+1])
        
        plan_time = time.time() - plan_time
        
        print(f"Cost: {cost:.3f} (minimized travel time)")
        print(f"Planning Time: {plan_time:.3f}s")
        print(f"Path length: {len(plan)} steps")
        print(f"Final distance to goal: {np.linalg.norm(current - goal_config):.3f}")

        return np.hstack(plan)