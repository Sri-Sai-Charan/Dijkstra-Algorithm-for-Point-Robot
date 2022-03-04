#usr/bin/python3
from cv2 import bitwise_and
import numpy as np
import cv2 as cv

class nodes():
    def __init__(self,position):
        self.position = position
        self.cost = np.inf
        self.parent = [0,0]

visited_map = np.zeros((252,402),dtype='uint16')
cost_map = np.ones((252,402),dtype=float)*1000
parent_map = np.zeros((252,402,2),dtype='uint16')
class lists():
    def __init__(self):
        self.OpenNodes = []      

class action():
    def __init__(self):
        self.action_sets= [(1,0),(-1,0), (0,1), (0,-1), (1,1), (-1,1),(1,-1),(-1,-1)]
        self.cost = [1,1,1,1,1.4,1.4,1.4,1.4]

def back_track(start,current,my_map):
    optimum_path = [[int(current.position[0]),int(current.position[1])]]

    while True:
        idx = len(optimum_path)-1
        optimum_path.append(parent_map[optimum_path[idx][0],optimum_path[idx][1]])
        if (optimum_path[idx][0] == start.position[0] and optimum_path[idx][1] == start.position[1]) :
            break
        
    optimum_path_map = my_map.copy()
    while bool(optimum_path):
        idx = optimum_path.pop()
        optimum_path_map[idx[0],idx[1]] = [0,0,255]
        # cv.imshow("Optimum Path Generation",optimum_path_map)
        # cv.waitKey(1)
    cv.imshow("Optimum Path",optimum_path_map)
    cv.waitKey(2)

def DijkstraAlogrithm(start,goal,my_map):

    current = start
    my_lists = lists()
    my_lists.OpenNodes.append(current)
    path_map = my_map.copy()

    while True :
        check_future_state(current,my_map,my_lists,path_map)
        if current.position == goal.position:
            break
        current = my_lists.OpenNodes[0]
        # cv.imshow("Path",path_map)
        # cv.waitKey(1)
    back_track(start,current,path_map)   
    print("Goal reached")
    
def check_validity_position(node,my_map):
    position1 = node.position
    # position2 = goal.position
    if (int(my_map[position1[0],position1[1],1])==255):
        return False
    else:
        return True

def hashmap_implementation(future):
    if visited_map[future.position[0],future.position[1]]==1:
        if cost_map[future.position[0],future.position[1]] > future.cost:
            cost_map[future.position[0],future.position[1]] = future.cost
            parent_map[future.position[0],future.position[1]] = future.parent

        return False
    else:
        cost_map[future.position[0],future.position[1]] = future.cost
        visited_map[future.position[0],future.position[1]] = 1
        parent_map[future.position[0],future.position[1]] = future.parent
        return True

def check_future_state(current,my_map,my_lists,path_map):
    actions = action()
    
    for ite in range(8):
        move = [0,0]
        move[0] = current.position[0] + actions.action_sets[ite][0]
        move[1] = current.position[1] + actions.action_sets[ite][1]
        cost = 0
        cost = current.cost + actions.cost[ite]
        
        if (int(my_map[move[0],move[1],1])!=255):
            
            future = nodes([0,0])
            future.position = move
            future.cost = cost
            future.parent = current.position
            if hashmap_implementation(future):
                my_lists.OpenNodes.append(future)
                path_map[future.position[0],future.position[1]]=[0,255,0]                

    my_lists.OpenNodes.pop(0) 
    my_lists.OpenNodes = sorted(my_lists.OpenNodes,key=lambda x: x.cost , reverse=False)
    
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

    #Circle
    circle_map = np.zeros_like(my_map)
    for i in range(64 - 45, 64 + 45):
        for j in range(299 - 42, 299 + 42):
            if (i - 64) **2 + (j - 299)**2 <= 45**2:
                circle_map[i,j] = obstacle_color

    #Hexagon
    hexagon_map = np.zeros_like(my_map)       
    hexagon_pts= np.array([[164,170], #p1
                            [199,190], #p2
                            [234,170], #p3
                            [234,130], #p4
                            [199,110], #p5
                            [164,130]],float) #p6
                        
    side1=half_planes(hexagon_map,hexagon_pts[0],hexagon_pts[1],obstacle_color,False)
    side2=half_planes(hexagon_map,hexagon_pts[1],hexagon_pts[2],obstacle_color,True)
    side2 = cv.bitwise_and(side2,side1)
    side3=half_planes(hexagon_map,hexagon_pts[2],hexagon_pts[3],obstacle_color,True)
    side4=half_planes(hexagon_map,hexagon_pts[3],hexagon_pts[4],obstacle_color,True)
    side5=half_planes(hexagon_map,hexagon_pts[4],hexagon_pts[5],obstacle_color,False)
    side5=cv.bitwise_and(side5,side4)
    side6=half_planes(hexagon_map,hexagon_pts[5],hexagon_pts[0],obstacle_color,False)
    side6 = cv.bitwise_and(side3,side6)
    my_map = cv.bitwise_and(side2,side5)
    hexagon_map = cv.bitwise_and(my_map,side6)

    #Other Obstace
    other_obstacle_map = np.zeros_like(my_map)  
    other_obstacle= np.array([[36,65], #p1
                            [105,150], #p2
                            [90,70], #p3
                            [115,40]],float) #p4
    side1=half_planes(other_obstacle_map,other_obstacle[0],other_obstacle[1],obstacle_color,False)
    side2=half_planes(other_obstacle_map,other_obstacle[1],other_obstacle[3],obstacle_color,True)
    side3=half_planes(other_obstacle_map,other_obstacle[3],other_obstacle[0],obstacle_color,False)
    side2 = cv.bitwise_and(side1,side2)
    side3 = cv.bitwise_and(side3,side2)
    mask1 = half_planes(other_obstacle_map,other_obstacle[1],other_obstacle[2],obstacle_color,True)
    mask2 = half_planes(other_obstacle_map,other_obstacle[2],other_obstacle[3],obstacle_color,True)
    mask3 = half_planes(other_obstacle_map,other_obstacle[3],other_obstacle[1],obstacle_color,False)
    mask1 = cv.bitwise_or(mask2,mask1)
    mask3 = cv.bitwise_or(mask3,mask1)
    other_obstacle_map=cv.bitwise_and(mask3,side3)
    my_map = cv.bitwise_or(other_obstacle_map,hexagon_map)
    my_map = cv.bitwise_or(my_map,circle_map)

    #Border
    my_map[0,:]=obstacle_color
    my_map[:,401]=obstacle_color
    my_map[251,:]=obstacle_color
    my_map[:,0] = obstacle_color
    # cv.imshow("Obstacle Space",my_map)
    # cv.waitKey(0)
    return my_map

import random

def testing_all_test_cases():
    global visited_map 
    global cost_map 
    global parent_map 
    for ite in range(100):
        start_x = random.randint(1, 250)
        start_y = random.randint(1,400)
        goal_x = random.randint(1,250)
        goal_y = random.randint(1,400)

        visited_map = np.zeros((252,402),dtype='uint16')
        cost_map = np.ones((252,402),dtype=float)*1000
        parent_map = np.zeros((252,402,2),dtype='uint16')
        my_map = np.zeros((252,402,3),dtype='uint8')
        obstacle_color = [255,255,255]
        my_map = PopulateMap(my_map,obstacle_color)
        start_node = nodes([start_x,start_y])
        start_node.cost = 0
        start_node.parent = [None,None]
        goal_node = nodes([goal_x,goal_y])
        cost_map[start_x,start_y] = 0.0
        visited_map[start_x,start_y] = 1
        parent_map[start_x,start_y] = [start_x,start_y]
        print(start_x,start_y,goal_x,goal_y)
        if check_validity_position(start_node,my_map):
            if check_validity_position(goal_node,my_map):
                print("calling dijkstra")
                DijkstraAlogrithm(start_node,goal_node,my_map)
            else:
                goal_x +=1
                goal_y +=1
                
        else:
            start_x +=1
            start_y +=1
def main():
    # my_map = np.zeros((252,402,3),dtype='uint8')
    # obstacle_color = [255,255,255]
    # my_map = PopulateMap(my_map,obstacle_color)
    # start_x = 2
    # start_y = 2
    # goal_x = 200
    # goal_y = 341
    # start_node = nodes([start_x,start_y])
    # start_node.cost = 0
    # start_node.parent = [None,None]
    # goal_node = nodes([goal_x,goal_y])
    # cost_map[start_x,start_y] = 0.0
    # visited_map[start_x,start_y] = 1
    # parent_map[start_x,start_y] = [start_x,start_y]
    # print(start_x,start_y,goal_x,goal_y)

    # if check_validity_position(start_node,my_map):
    #     if check_validity_position(goal_node,my_map):
    #         DijkstraAlogrithm(start_node,goal_node,my_map)
    testing_all_test_cases()
    
if __name__ == '__main__':
    main()
