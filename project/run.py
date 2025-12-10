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
        print("Which map?")
        print("Options: ")
        print("Map Number 1")
        print("Map Number 2")
        print("Map Number 3")
        print("Map Number 4")
        print("Map Number 5")
        print("Map Number 6")
        print("Map Number 7")
        print("Map Number 8")
        print("Map Number 9")

        audio = r.listen(source)

    try:
        text2 = r.recognize_google(audio).lower()
    except:
        print("Could not understand map choice")
        exit(0)

    speech_to_map = {
        "map number one": ["map1.txt", [0.0, 0.0], [8.0, 8.0]],
        "map number two": ["map2.txt", [321.0, 148.0], [106.0, 202.0]],
        "map number three": ["map3.txt", [0.0, 0.0], [18.0, 18.0]],
        "map number four": ["map4.txt", [2.0, 2.0], [22.0, 22.0]],
        "map number five": ["map5.txt", [1.0, 1.0], [795.0, 795.0]],
        "map number six": ["map6.txt", [2.0, 2.0], [97.0, 97.0]],
        "map number seven": ["map7.txt", [1.0, 1.0], [28.0, 28.0]],
        "map number eight": ["map8.txt", [2.0, 2.0], [27.0, 27.0]],
        "map number nine": ["map9.txt", [0.0, 0.0], [799.0, 799.0]]
    }

    if text2 not in speech_to_map:
        print("Invalid map selection")
        exit(0)

    map_file, start_pos, goal_pos = speech_to_map[text2]
    start = np.array(start_pos).reshape(2, 1)
    goal = np.array(goal_pos).reshape(2, 1)

    # ---- Choose Epsilon ----
    with sr.Microphone() as source:
        print("Say epsilon value. Options :")
        print("Number One for 1")
        print("Number Two for 2")
        print("Number Three for 3")
        audio = r.listen(source)

    try:
        text3 = r.recognize_google(audio).lower()
    except:
        print("Could not understand epsilon")
        exit(0)

    speech_to_value = {
        "number one": 1,
        "number two": 2,
        "number three": 3,
    }

    if text3 not in speech_to_value:
        print("Invalid epsilon value")
        exit(0)

    epsilon = speech_to_value[text3]

    # ---- Choose Planner ----
    with sr.Microphone() as source:
        print("Which planner?")
        print("Say 'Number 1' for Astar")
        print("Say 'Number 2' for Theta")
        audio = r.listen(source)

    try:
        text4 = r.recognize_google(audio).lower()
    except:
        print("Could not understand planner choice")
        exit(0)

    # ---- Setup Environment ----
    planning_env = MapEnvironment(map_file, start, goal)

    # ---- Create Planner ----
    if "one" in text4:
        planner = AStarPlanner(planning_env, epsilon)
        planner_type = "astar"
    else:
        planner = ThetaStarPlanner(planning_env, epsilon)
        planner_type = "thetastar"

    # ---- Run ----
    main(planning_env, planner, start, goal, planner_type)
