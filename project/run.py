import argparse
import numpy as np
import matplotlib.pyplot as plt

from MapEnvironment import MapEnvironment
from AStarPlanner import AStarPlanner
import speech_recognition as sr

r = sr.Recognizer()

def main(planning_env, planner, start, goal, argplan = 'astar'):

    # Notify.
    input('Press any key to begin planning...')

    planning_env.init_visualizer()

    # Plan.
    plan = planner.Plan(start, goal)

    # Visualize the final path.
    tree = None
    visited = planner.visited
        
    planning_env.visualize_plan(plan, tree, visited)
    plt.show()


if __name__ == "__main__":
    map_file = "map1.txt"
    planner_type="astar"
    start = np.array([0.0, 0.0]).reshape(2, 1)
    goal = np.array([8.0, 8.0]).reshape(2, 1)
    epsilon = 1.0

    # Determine whether or not to run the code
    with sr.Microphone() as source:
        print("Speak now.")
        audio = r.listen(source)

    try:
        text1 = r.recognize_google(audio)
    except sr.UnknownValueError:
        print("Could not understand audio")
    except sr.RequestError:
        print("API unavailable")

    if text1 in ["start", "begin", "activate", "commence", "initiate" ," launch"]:
        print("Starting the program. Please select the following parameters.")
    else:
        print("Ending the program.")
        exit(0)

    # Determine what map to use
    with sr.Microphone() as source:
        print("What map to use? - [map one, map two, map 3]")
        audio = r.listen(source)

    try:
        text2 = r.recognize_google(audio)
    except sr.UnknownValueError:
        print("Could not understand audio")
    except sr.RequestError:
        print("API unavailable")

    speech_to_map = {
        "map one" : ["map1.txt", [0.0, 0.0], [8.0, 8.0]],
        "map two" : ["map2.txt", [321.0, 148.0], [106.0, 202.0]],
        "map three" : ["map3.txt", [0.0, 0.0], [18.0, 18.0]],
    }

    map_details = speech_to_map[text2]
    map_file = map_details[0]
    start = map_details[1]
    end = map_details[2]

    start = np.array(start).reshape(2, 1)
    goal = np.array(end).reshape(2, 1)

    # Determine what elipson value to use
    with sr.Microphone() as source:
        print("What elipson value to use?")
        audio = r.listen(source)

    try:
        text3 = r.recognize_google(audio)
    except sr.UnknownValueError:
        print("Could not understand audio")
    except sr.RequestError:
        print("API unavailable")
    
    speech_to_value = {
        "one" : 1,
        "two" : 2,
        "three" : 3,
    }

    epsilon = speech_to_value[text3]

    # First setup the environment and the robot.
    planning_env = MapEnvironment(map_file, start, goal)

    planner = AStarPlanner(planning_env, epsilon)

    main(planning_env, planner, start, goal, planner)