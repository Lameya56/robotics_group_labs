# Pathfinding Visualization with A* and Theta*

A voice-controlled pathfinding program that demonstrates A* and Theta* algorithms on various map environments.

## Prerequisites

Install the required dependencies:

```bash
pip install numpy matplotlib SpeechRecognition pyaudio
```

## How to Run

Simply execute the program:

```bash
python run.py
```

## Voice Control Instructions

The program uses voice commands for all interactions. Follow the prompts:

1. **Start Trigger**: Say one of the following to begin:
   - "start"
   - "begin"
   - "activate"
   - "commence"
   - "initiate"
   - "launch"

2. **Select a Map**: Choose from 9 available maps by saying:
   - "Map Number One" through "Map Number Nine"

   **Available Maps:**
   - Map 1: 10x10 basic map (start: [0,0], goal: [8,8])
   - Map 2: 512x512 complex map (start: [321,148], goal: [106,202])
   - Map 3: 20x20 map (start: [0,0], goal: [18,18])
   - Map 4: 25x25 map (start: [2,2], goal: [22,22])
   - Map 5: 800x800 large map (start: [1,1], goal: [795,795])
   - Map 6: 100x100 map (start: [2,2], goal: [97,97])
   - Map 7: 30x30 map (start: [1,1], goal: [28,28])
   - Map 8: 30x30 map (start: [2,2], goal: [27,27])
   - Map 9: 800x800 map (start: [0,0], goal: [799,799])

3. **Choose Epsilon Value**: Say the epsilon parameter for the planner:
   - "Number One" for epsilon = 1
   - "Number Two" for epsilon = 2
   - "Number Three" for epsilon = 3

4. **Select Planner**: Choose the pathfinding algorithm:
   - "Number 1" for A* algorithm
   - "Number 2" for Theta* algorithm

5. **Begin Planning**: Press any key when prompted to start the pathfinding visualization.

## Output

The program will:
- Display the pathfinding process
- Show visited nodes
- Visualize the final path
- Display the chosen algorithm in the plot title

## Using the RL Agent on 2D Grid Environments

Installing Required Dependences:

pip install torch numpy matplotlib

Parameters:
- map: (.txt) map name
- start: (tuple) start coordinates of agent
- randomize_goals: (flag) use random valid goals
- end: (tuple) end coordinates of agent *replace with randomize_goals flag
- episodes: (int) amount of episodes agent will train on
- plot_live: (flag) will plot the episode results live while training
- save_plot: (.png) save the results of the completed training by reward value and success rate to this file name
- model_out: (.pth) output agent to this file name
- model: (.pth) reference an existing model file path to evaluate
- hidden_size: (int) modify the size of the neural network hidden layer
- plot_rewards: (flag) plot the reward value of the results live
- save_plot: (.png) save the plot to this file name
- render: (flag) save each episode's results in a gif
- gif: (.gif) save the gif to this file name
- model_in: (.pth) reference and existing model file path to retrain

### Example of Creating + Testing an Agent

Training a New Agent:

python3 train_grid_agent.py --map map3.txt --start 0,0 --randomize_goals --episodes 500 --plot_live --save_plot training_map3.png --model_out model_map3.pth

Evaluating the Agent's training:

python3 eval_grid_agent.py --map map3.txt --model model_map3.pth --hidden_size 256 --randomize_goals --episodes 5 --plot_rewards --save_plot eval_map3.png --render --gif demo.gif

Retraining an Existing Agent:

python3 train_grid_agent.py --map map3.txt --model_in model_map3.pth --randomize_goals --episodes 500 --plot_live --save_plot training_map3_continued.png --model_out model_map3.pth
