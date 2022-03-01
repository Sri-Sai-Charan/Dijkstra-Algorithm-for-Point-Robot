#usr/bin/python3
from cv2 import bitwise_and
import numpy as np
import matplotlib.pyplot as plt
# import py
import cv2 as cv
class nodes():
    def __init__(self,position):
        self.position = position
        self.cost = np.inf
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
        if current.position == goal.position:
            BackTrace()
            break
        else:
            check_future_state(current,my_map,my_lists)
            my_lists.ClosedNodes.append(current)
            break

def check_visited(visited,future):
    for ite in range(len(visited)):
        if (visited[ite].position==future.position).all():
            if visited[ite].cost>future.cost:
                visited[ite].cost = future.cost
            return False
        else:
            visited.append(future)
            return True

def check_future_state(current,my_map,my_lists):
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
            # print(current.position,future.position,future.cost)
            my_lists.OpenNodes.append(future)
    my_lists.OpenNodes.pop(0)

def BackTrace():
    temp = 0

def half_planes(my_map,point1,point2,obstacle_color,upper):
    m = (point2[0]-point1[0])/(point2[1]-point1[1]+(1e-6))
    temp=np.zeros_like(my_map)
    for y  in range(1,400):
        c = point1[0] - m*point1[1]
        for x in range(1,250):
            if upper :
                if (y < ((m*x)+c)):
                    temp[x,y]= obstacle_color
            else:
                if (y > ((m*x)+c)):
                    temp[x,y]= obstacle_color
    return temp

def PopulateMap(my_map,obstacle_color):
    #Border
    my_map[0,:]=obstacle_color
    my_map[:,401]=obstacle_color
    my_map[251,:]=obstacle_color
    my_map[:,0] = obstacle_color

    #Circle
    circle_map = np.zeros_like(my_map)
    for i in range(64 - 42, 64 + 42):
        for j in range(299 - 42, 299 + 42):
            if (i - 64) **2 + (j - 299)**2 <= 42**2:
                circle_map[i,j] = obstacle_color

    #Hexagon
    hexagon_map = np.zeros_like(my_map)       
    hexagon_pts= np.array([[164,170], #p1
                            [199,190], #p2
                            [234,170], #p3
                            [234,130], #p4
                            [199,110], #p5
                            [164,130]],np.float) #p6
                        
    side1=half_planes(hexagon_map,hexagon_pts[0],hexagon_pts[1],obstacle_color,False)
    side2=half_planes(hexagon_map,hexagon_pts[1],hexagon_pts[2],obstacle_color,True)
    side2 = side2 * side1
    side3=half_planes(hexagon_map,hexagon_pts[2],hexagon_pts[3],obstacle_color,True)
    side4=half_planes(hexagon_map,hexagon_pts[3],hexagon_pts[4],obstacle_color,True)
    side5=half_planes(hexagon_map,hexagon_pts[4],hexagon_pts[5],obstacle_color,False)
    side5=side5*side4
    side6=half_planes(hexagon_map,hexagon_pts[5],hexagon_pts[0],obstacle_color,False)
    side6=side6*side3
    my_map = cv.bitwise_and(side2,side5)
    hexagon_map = cv.bitwise_and(my_map,side6)

    #Other Obstace
    other_obstacle_map = np.zeros_like(my_map)  
    other_obstacle= np.array([[36,65], #p1
                            [105,150], #p2
                            [90,70], #p3
                            [115,40]],np.float) #p4
    side1=half_planes(other_obstacle_map,other_obstacle[0],other_obstacle[1],obstacle_color,False)
    side2=half_planes(other_obstacle_map,other_obstacle[1],other_obstacle[3],obstacle_color,True)
    side3=half_planes(other_obstacle_map,other_obstacle[3],other_obstacle[0],obstacle_color,False)
    side2 = side1*side2
    side3 = cv.bitwise_and(side3,side2)
    mask1 = half_planes(other_obstacle_map,other_obstacle[1],other_obstacle[2],obstacle_color,True)
    mask2 = half_planes(other_obstacle_map,other_obstacle[2],other_obstacle[3],obstacle_color,True)
    mask3 = half_planes(other_obstacle_map,other_obstacle[3],other_obstacle[1],obstacle_color,False)
    mask1 = cv.bitwise_or(mask2,mask1)
    mask3 = cv.bitwise_or(mask3,mask1)
    other_obstacle_map=cv.bitwise_and(mask3,side3)

    my_map = cv.bitwise_or(other_obstacle_map,hexagon_map)
    my_map = cv.bitwise_or(my_map,circle_map)
    cv.imshow("Obstacle Space",my_map)
    cv.waitKey(0)
def main():
    my_map = np.zeros((252,402,3))
    obstacle_color = [255,255,255]
    PopulateMap(my_map,obstacle_color)
    start_node = nodes([2,2])
    goal_node = nodes([250,400])
    if check_validity_position(start_node,my_map):
        if check_validity_position(goal_node,my_map):
            DijkstraAlogrithm(start_node,goal_node,my_map)

    
if __name__ == '__main__':
    main()
