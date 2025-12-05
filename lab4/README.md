# Robotics Lab 4
### How to Run Our Code

Main Code
```
python3 run.py -m <map_file> -p <planner_name> -s <start_x> <start_y> -g <goal_x> <goal_y>
```

Available Map_Files
- map1.txt
- map2.txt
- map3.txt
- map4.txt 
- map5.txt
- map6.txt

Available Planner Names
- dfs
- bfs
- iddfs
- astar
- rrt
- prm
- bug
- potentialfields
- visibilitygraph

# Map 1
### DFS
python3 run.py -m map1.txt -p dfs -s 0 0 -g 8 8

### BFS
python3 run.py -m map1.txt -p bfs -s 0 0 -g 8 8

### IDDFS
python3 run.py -m map1.txt -p iddfs -s 0 0 -g 8 8

### A*
python3 run.py -m map1.txt -p astar -s 0 0 -g 8 8 --epsilon 1

### RRT
python3 run.py -m map1.txt -p rrt -s 0 0 -g 8 8 --eta 1 --bias 0.05

### PRM
python3 run.py -m map1.txt -p prm -s 0 0 -g 8 8 --num_samples 100 --num_neighbors 5

### Potential 
python3 run.py -m map1.txt -p potentialfields -s 0 0 -g 8 8

### Visibility
python3 run.py -m map1.txt -p visibilitygraph -s 0 0 -g 8 8

### Bug
python3 run.py -m map1.txt -p bug -s 0 0 -g 8 8



# Map 2
### DFS
python3 run.py -m map2.txt -p dfs -s 321 148 -g 106 202

### BFS
python3 run.py -m map2.txt -p bfs -s 321 148 -g 106 202

### IDDFS
python3 run.py -m map2.txt -p iddfs -s 321 148 -g 106 202

### A*
python3 run.py -m map2.txt -p astar -s 321 148 -g 106 202 --epsilon 1

### RRT
python3 run.py -m map2.txt -p rrt -s 321 148 -g 106 202 --eta 5 --bias 0.1

### PRM
python3 run.py -m map2.txt -p prm -s 321 148 -g 106 202 --num_samples 500 --num_neighbors 10

### Potential 
python3 run.py -m map2.txt -p potentialfields -s 321 148 -g 106 202

### Visibility
python3 run.py -m map2.txt -p visibilitygraph -s 321 148 -g 106 202

### Bug
python3 run.py -m map2.txt -p bug -s 321 148 -g 106 202

# Map 3
### DFS
python3 run.py -m map3.txt -p dfs -s 0 0 -g 18 18

### BFS
python3 run.py -m map3.txt -p bfs -s 0 0 -g 18 18

### IDDFS
python3 run.py -m map3.txt -p iddfs -s 0 0 -g 18 18

### A*
python3 run.py -m map3.txt -p astar -s 0 0 -g 18 18 --epsilon 1

### RRT
python3 run.py -m map3.txt -p rrt -s 0 0 -g 18 18 --eta 1.5 --bias 0.05

### PRM
python3 run.py -m map3.txt -p prm -s 0 0 -g 18 18 --num_samples 200 --num_neighbors 8

### Potential
python3 run.py -m map3.txt -p potentialfields -s 0 0 -g 18 18

### Visibility
python3 run.py -m map3.txt -p visibilitygraph -s 0 0 -g 18 18

### Bug
python3 run.py -m map3.txt -p bug -s 0 0 -g 18 18

# Map 4
### DFS
python3 run.py -m map4.txt -p dfs -s 1 1 -g 500 501

### BFS
python3 run.py -m map4.txt -p bfs -s 1 1 -g 500 501

### IDDFS
python3 run.py -m map4.txt -p iddfs -s 1 1 -g 500 501

### A*
python3 run.py -m map4.txt -p astar -s 1 1 -g 500 501 --epsilon 1

### RRT
python3 run.py -m map4.txt -p rrt -s 1 1 -g 500 501 --eta 10 --bias 0.1

### PRM
python3 run.py -m map4.txt -p prm -s 1 1 -g 500 501 --num_samples 1000 --num_neighbors 15

### Potential
python3 run.py -m map4.txt -p potentialfields -s 1 1 -g 500 501

### Visibility
python3 run.py -m map4.txt -p visibilitygraph -s 1 1 -g 500 501

### Bug
python3 run.py -m map4.txt -p bug -s 1 1 -g 500 501

# Map 5
### DFS
python3 run.py -m map5.txt -p dfs -s 1 1 -g 24 24

### BFS
python3 run.py -m map5.txt -p bfs -s 1 1 -g 24 24

### IDDFS
python3 run.py -m map5.txt -p iddfs -s 1 1 -g 24 24

### A*
python3 run.py -m map5.txt -p astar -s 1 1 -g 24 24 --epsilon 1

### RRT
python3 run.py -m map5.txt -p rrt -s 1 1 -g 24 24 --eta 2 --bias 0.05

### PRM
python3 run.py -m map5.txt -p prm -s 1 1 -g 24 24 --num_samples 200 --num_neighbors 8

### Potential
python3 run.py -m map5.txt -p potentialfields -s 1 1 -g 24 24

### Visibility
python3 run.py -m map5.txt -p visibilitygraph -s 1 1 -g 24 24

### Bug
python3 run.py -m map5.txt -p bug -s 1 1 -g 24 24

# Map 6
### DFS
python3 run.py -m map6.txt -p dfs -s 5 79 -g 200 130

### BFS
python3 run.py -m map6.txt -p bfs -s 5 79 -g 200 130

### IDDFS
python3 run.py -m map6.txt -p iddfs -s 5 79 -g 200 130

### A*
python3 run.py -m map6.txt -p astar -s 5 79 -g 200 130 --epsilon 1

### RRT
python3 run.py -m map6.txt -p rrt -s 5 79 -g 200 130 --eta 5 --bias 0.1

### PRM
python3 run.py -m map6.txt -p prm -s 5 79 -g 200 130 --num_samples 800 --num_neighbors 12

### Potential
python3 run.py -m map6.txt -p potentialfields -s 5 79 -g 200 130

### Visibility
python3 run.py -m map6.txt -p visibilitygraph -s 5 79 -g 200 130

### Bug
python3 run.py -m map6.txt -p bug -s 5 79 -g 200 130
