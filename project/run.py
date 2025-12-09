import argparse
import numpy as np
import matplotlib.pyplot as plt

from MapEnvironment import MapEnvironment
from AStarPlanner import AStarPlanner
from ThetaStarPlanner import ThetaStarPlanner
import speech_recognition as sr

r = sr.Recognizer()

def main(planning_env, planner, start, goal, planner_type='astar'):

    input('Press any key to begin planning...')

    planning_env.init_visualizer()

    # Run planner
    plan = planner.Plan(start, goal)

    # Visualize result
    tree = None
    visited = planner.visited
        
    planning_env.visualize_plan(plan, tree, visited)
    plt.title(planner_type.upper())
    plt.show()


if __name__ == "__main__":
    map_file = "map1.txt"
    planner_type = "astar"
    start = np.array([0.0, 0.0]).reshape(2, 1)
    goal = np.array([8.0, 8.0]).reshape(2, 1)
    epsilon = 1.0

    # ---- Start Voice Trigger ----
    with sr.Microphone() as source:
        print("Say 'start' to begin.")
        audio = r.listen(source)

    try:
        text1 = r.recognize_google(audio).lower()
    except:
        print("Could not understand audio")
        exit(0)

    if text1 not in ["start", "begin", "activate", "commence", "initiate", "launch"]:
        print("Ending the program.")
        exit(0)

    print("Starting the program...")

    # ---- Choose Map ----
    with sr.Microphone() as source:
        print("Which map? Say: map one, map two, map three, or map four")
        audio = r.listen(source)

    try:
        text2 = r.recognize_google(audio).lower()
    except:
        print("Could not understand map choice")
        exit(0)

    speech_to_map = {
        "map one": ["map1.txt", [0.0, 0.0], [8.0, 8.0]],
        "map two": ["map2.txt", [321.0, 148.0], [106.0, 202.0]],
        "map three": ["map3.txt", [0.0, 0.0], [18.0, 18.0]],
        "map four": ["map4.txt", [2.0, 2.0], [22.0, 22.0]],
    }

    if text2 not in speech_to_map:
        print("Invalid map selection")
        exit(0)

    map_file, start_pos, goal_pos = speech_to_map[text2]
    start = np.array(start_pos).reshape(2, 1)
    goal = np.array(goal_pos).reshape(2, 1)

    # ---- Choose Epsilon ----
    with sr.Microphone() as source:
        print("Say epsilon value: one, two, or three")
        audio = r.listen(source)

    try:
        text3 = r.recognize_google(audio).lower()
    except:
        print("Could not understand epsilon")
        exit(0)

    speech_to_value = {
        "one": 1,
        "two": 2,
        "three": 3,
    }

    if text3 not in speech_to_value:
        print("Invalid epsilon value")
        exit(0)

    epsilon = speech_to_value[text3]

    # ---- Choose Planner ----
    with sr.Microphone() as source:
        print("Which planner? Say 'astar' or 'theta star'")
        audio = r.listen(source)

    try:
        text4 = r.recognize_google(audio).lower()
    except:
        print("Could not understand planner choice")
        exit(0)

    # ---- Setup Environment ----
    planning_env = MapEnvironment(map_file, start, goal)

    # ---- Create Planner ----
    if "theta" in text4:
        planner = ThetaStarPlanner(planning_env, epsilon)
        planner_type = "thetastar"
    else:
        planner = AStarPlanner(planning_env, epsilon)
        planner_type = "astar"

    # ---- Run ----
    main(planning_env, planner, start, goal, planner_type)
