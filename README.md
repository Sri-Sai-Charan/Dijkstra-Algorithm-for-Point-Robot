# Dijkstra-Algorithm-for-Point-Robot
An individual project aimed at implementing Dijkstra's path planning algorithm on a point robot to navigate a map that contains obstacles

## Description

This code uses Dijkstra's path planning algorithm to traverse nodes and locates the goal node. It stops searching once the goal node is reched and begins backtracking. The backtracking is used to find the optimal path to be taken after Dijkstra's path planning algorithm has been implemented. OpenCV is used to visualize the path taken by the point robot and the most optimal path after asigning costs to each node.

This code employs hashmaps to reduce time complexity, by storing visited, cost and parents in a 2D mapped array. 

## Getting Started
* Enter your start node in main() by replacing the value assigned to "start_x , start_y"
* Enter your goal node in main() by replacing the value assigned to "goal_x , goal_y"
### Dependencies

* numpy library, python version 3.X.X needed before installing program.
* cv2 library, opencv is needed before running the program

### Installing

* Download the zip file \ pull all files and extract into any workspace as the code is self-contained and does not require any particular environment. 
* NOTE: test_random_cases will create 100 random pairs of start and goal nodes for Dijkstra to Compute and cv.waitKey(1) will limit each test case to a maximum complie time of 100 sec. You can comment lines 53, 54 to stop after each call of Dijkstra.

### Executing program

* Open your IDE in the parent folder of plot_path.py
* Open your text editor and ensure the python interpreter is at least python version 3.X.X and run the below comand (for VSC) or run the file
```
python3 Dijkstra-pathplanning-srisaicharan-velisetti.py
```

* Open your text editor and ensure the python interpreter is at least python version 3.X.X and run the below comand (for VSC) or run the file
```
CTRL + ALT + N
```

### Results

* Path Generation <br />
![alt text](https://github.com/Sri-Sai-Charan/Dijkstra-Algorithm-for-Point-Robot/blob/main/Results/path_gen.gif)

* Optimum Path Generation <br />
![alt text](https://github.com/Sri-Sai-Charan/Dijkstra-Algorithm-for-Point-Robot/blob/main/Results/opt_path_gen.gif)

## Authors

Sri Sai Charan Velisetti - svellise@umd.edu

