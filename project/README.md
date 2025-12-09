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
