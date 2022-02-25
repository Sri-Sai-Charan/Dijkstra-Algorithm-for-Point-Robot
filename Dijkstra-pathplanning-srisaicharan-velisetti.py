#usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
# import py

class nodes():
    def __init__(self,current_node,parent_node) -> None:
        self.current = current_node
        self.parent = parent_node
        # self.cost = 0

# class action_set():
#     def __init__(self) -> None:
#         self.

def move(current):
    current


def Dijstra(start,goal):
    if start.current == goal.current :
        print("Goal Reached")
    else:
        move(start)


def main():
    start_node = nodes((0,1),0)
    goal_node = nodes((10,10),None)

    action_sets= {(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1),(1,-1),(-1,-1)}
    cost = {1,1,1,1,1.4,1.4,1.4,1.4}
    Dijstra(start_node,goal_node)

if __name__ == '__main__':
    main()
