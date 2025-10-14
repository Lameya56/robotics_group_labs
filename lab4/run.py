import argparse
import numpy as np
import matplotlib.pyplot as plt

from MapEnvironment import MapEnvironment
from AStarPlanner import AStarPlanner
from RRTPlanner import RRTPlanner
from BugPlanner import BugPlanner
from BFSPlanner import BFSPlanner
from DFSPlanner import DFSPlanner
from IDDFSPlanner import IDDFSPlanner
from PRMPlanner import PRMPlanner
from PotentialFieldsPlanner import PotentialFieldsPlanner
from VisibilityGraphPlanner import VisibilityGraphPlanner


def main(planning_env, planner, start, goal, argplan = 'astar'):

    # Notify.
    input('Press any key to begin planning...')

    planning_env.init_visualizer()

    # Plan.
    plan = planner.Plan(start, goal)

    # Visualize the final path.
    tree = None
    visited = None
    if argplan in ['rrt', 'prm']:
        tree = planner.tree
    elif argplan in ['astar', 'bfs', 'dfs', 'iddfs']:
        visited = planner.visited
        
    planning_env.visualize_plan(plan, tree, visited)
    plt.show()


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-m', '--map', type=str, default='map1.txt',
                        help='The environment to plan on')    
    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='The planner to run (astar, rrt, bug, bfs, dfs, iddfs, prm, potentialfields, visibilitygraph)')
    parser.add_argument('-s', '--start', nargs='+', type=float, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=float, required=True)
    
    # A* arguments
    parser.add_argument('-eps', '--epsilon', type=float, default=1.0, help='Epsilon for A*')

    # RRT arguments
    parser.add_argument('-eta', '--eta', type=float, default=1.0, help='eta for RRT')
    parser.add_argument('-b', '--bias', type=float, default=0.05, help='Goal bias for RRT')

    # PRM arguments
    parser.add_argument('--num_samples', type=int, default=100, help='Number of samples for PRM')
    parser.add_argument('--num_neighbors', type=int, default=5, help='Number of neighbors to connect for PRM')


    args = parser.parse_args()

    # First setup the environment and the robot.
    dim = 2
    args.start = np.array(args.start).reshape(dim, 1)
    args.goal = np.array(args.goal).reshape(dim, 1)
    planning_env = MapEnvironment(args.map, args.start, args.goal)

    # Next setup the planner
    if args.planner == 'astar':
        planner = AStarPlanner(planning_env, args.epsilon)
    elif args.planner == 'rrt':
        planner = RRTPlanner(planning_env, bias=args.bias, eta=args.eta)
    elif args.planner == 'bug':
        planner = BugPlanner(planning_env)
    elif args.planner == 'bfs':
        planner = BFSPlanner(planning_env)
    elif args.planner == 'dfs':
        planner = DFSPlanner(planning_env)
    elif args.planner == 'iddfs':
        planner = IDDFSPlanner(planning_env)
    elif args.planner == 'prm':
        planner = PRMPlanner(planning_env, args.num_samples, args.num_neighbors)
    elif args.planner == 'potentialfields':
        planner = PotentialFieldsPlanner(planning_env)
    elif args.planner == 'visibilitygraph':
        planner = VisibilityGraphPlanner(planning_env)
    else:
        print('Unknown planner option: %s' % args.planner)
        exit(0)

    main(planning_env, planner, args.start, args.goal, args.planner)