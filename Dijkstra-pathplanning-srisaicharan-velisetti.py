#usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
# import py
import cv2 as cv
class nodes():
    def __init__(self,position):
        self.position = position
        self.cost = 0
        self.parent = (0,0)

class lists():
    def __init__(self):
        self.OpenNodes = []
        self.ClosedNodes = []

class action():
    def __init__(self):
        self. action_sets= [(1,0),(-1,0), (0,1), (0,-1), (1,1), (-1,1),(1,-1),(-1,-1)]
        self.cost = [1,1,1,1,1.4,1.4,1.4,1.4]

def check_validity_position(node,my_map):
    position = node.position
    # print(position[0],position[1])
    if (my_map[position[0],position[1],:]==[255,255,255]).all():
        print("Invalid Start or Goal node")
        return False
    else:
        # print("Valid Goal and Start Nodes")
        return True

def DijkstraAlogrithm(start,goal,my_map):
    current = start
    my_lists = lists()
    while current.position != goal.position :
        my_lists.OpenNodes.append(current)
        x = my_lists.OpenNodes[0]
        my_lists.ClosedNodes.append(x)
        if x.position == goal.position:
            BackTrace()
            break
        else:
            check_future_state(current,my_map)
            break

def check_visited(visited,current):
    for ite in range(len(visited)):
        if (visited[ite].position==current.position).all():
            return False
        else:
            visited.append(current)

            return True

def check_future_state(current,my_map):
    actions = action()
    for ite in range(0,8):
        move = [0,0]
        move[0] = current.position[0] + actions.action_sets[ite][0]
        move[1] = current.position[1] + actions.action_sets[ite][1]
        cost = current.cost
        cost +=  actions.cost[ite]
        if (my_map[move[0],move[1]]!=[255,255,255]).all():
            future = nodes([0,0])
            future.position = move
            future.cost = cost
            print(future.position,future.cost)


def BackTrace():
    temp = 0


def PopulateMap(my_map,obstacle_color):
    # border
    my_map[0,:]=obstacle_color
    my_map[:,401]=obstacle_color
    my_map[251,:]=obstacle_color
    my_map[:,0] = obstacle_color
    #Circle
    for i in range(64 - 42, 64 + 42):
        for j in range(299 - 42, 299 + 42):
            if (i - 64) **2 + (j - 299)**2 <= 42**2:
                my_map[i,j] = obstacle_color
            
    hexagonpts= np.array([[164,170],
                            [199,190],
                            [234,170],
                            [234,130],
                            [199,110],
                            [164,130]],np.int32)
    other_obstacle= np.array([[36,65],
                            [105,150],
                            [90,70],
                            [115,40]],np.int32)
    pts=hexagonpts.reshape((-1,1,2))
    cv.fillPoly(my_map,[pts],(obstacle_color),1)
    pts=other_obstacle.reshape((-1,1,2))
    cv.fillPoly(my_map,[pts],(obstacle_color),1)

    cv.imshow('map',my_map)
    cv.waitKey(0)

def main():
    my_map = np.zeros((252,402,3))
    obstacle_color = [255,255,255]
    PopulateMap(my_map,obstacle_color)
    start_node = nodes([2,2])
    goal_node = nodes([250,400])
    a = action()
    # a.action_sets[1][0] += 1
    if check_validity_position(start_node,my_map):
        if check_validity_position(goal_node,my_map):
            DijkstraAlogrithm(start_node,goal_node,my_map)
    # print(a.action_sets[1][0]+1)
    
if __name__ == '__main__':
    main()
