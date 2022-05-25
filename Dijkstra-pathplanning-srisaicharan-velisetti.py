#usr/bin/python3
from cv2 import bitwise_and
import numpy as np
import cv2 as cv

class nodes():
    def __init__(self,position):
        self.position = position
        self.cost = np.inf
        self.parent = [0,0]

visited_map = np.zeros((250,400),dtype='uint16')
cost_map = np.ones((250,400),dtype=float)*1000
parent_map = np.zeros((250,400,2),dtype='uint16')
class lists():
    def __init__(self):
        self.OpenNodes = []      
        self.visited_list = []
class action():
    def __init__(self):
        self.action_sets= [(1,0),(-1,0), (0,1), (0,-1), (1,1), (-1,1),(1,-1),(-1,-1)]
        self.cost = [1,1,1,1,1.4,1.4,1.4,1.4]

def back_track(start,current,path_map,my_map,visited_list):
    frame = 0

    path_map = my_map.copy()
    visited_list.reverse()


    vid_path_gen = cv.VideoWriter('./path_gen.avi',cv.VideoWriter_fourcc('M','J','P','G'), 30, (400,250))
    vid_opt_path_gen = cv.VideoWriter('./opt_path_gen.avi',cv.VideoWriter_fourcc('M','J','P','G'), 30, (400,250))


    while bool(visited_list):
        
        idx = visited_list.pop()
        path_map[idx[0],idx[1]] = [0,255,0]
        if (frame % 100) ==0 :
            cv.imshow("Path Generation",path_map)
            vid_path_gen.write(path_map)
            cv.waitKey(1)
        frame +=1
    optimum_path = [[int(current.position[0]),int(current.position[1])]]

    while True:
        idx = len(optimum_path)-1
        optimum_path.append(parent_map[optimum_path[idx][0],optimum_path[idx][1]])
        if (optimum_path[idx][0] == start.position[0] and optimum_path[idx][1] == start.position[1]) :
            break

    optimum_path_map = path_map.copy()

    while bool(optimum_path):
        idx = optimum_path.pop()
        optimum_path_map[idx[0],idx[1]] = [0,0,255]
        cv.imshow("Optimum Path Generation",optimum_path_map)
        vid_opt_path_gen.write(optimum_path_map)
        cv.waitKey(1)
        
    cv.imshow("Optimum Path",optimum_path_map)
    cv.waitKey(0)

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
    back_track(start,current,path_map,my_map,my_lists.visited_list)   
    print("Goal reached")
    
def check_validity_position(node,my_map):
    position1 = node.position
    if not (position1[0] > 249 | position1[1] > 399) :
        if(int(my_map[position1[0],position1[1],1])>127):
            return False
        else:
            return True
    else:
        return False

def hashmap_implementation(future,visited_list):
    if visited_map[future.position[0],future.position[1]]==1:
        if cost_map[future.position[0],future.position[1]] > future.cost:
            cost_map[future.position[0],future.position[1]] = future.cost
            parent_map[future.position[0],future.position[1]] = future.parent

        return False
    else:
        cost_map[future.position[0],future.position[1]] = future.cost
        visited_map[future.position[0],future.position[1]] = 1
        parent_map[future.position[0],future.position[1]] = future.parent
        visited_list.append([future.position[0],future.position[1]])
        return True

def check_future_state(current,my_map,my_lists,path_map):
    actions = action()
    
    for ite in range(8):
        move = [0,0]
        move[0] = current.position[0] + actions.action_sets[ite][0]
        move[1] = current.position[1] + actions.action_sets[ite][1]
        cost = 0
        cost = current.cost + actions.cost[ite]
        
        if (int(my_map[move[0],move[1],1])<127):
            
            future = nodes([0,0])
            future.position = move
            future.cost = cost
            future.parent = current.position
            if hashmap_implementation(future,my_lists.visited_list):
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
                if (y <= ((m*x)+c)):
                    temp[x,y]= obstacle_color
            else:
                if (y >= ((m*x)+c)):
                    temp[x,y]= obstacle_color
    return temp

def PopulateMap(my_map,obstacle_color,tolerance=0):

    #Circle
    circle_map = np.zeros_like(my_map)
    for i in range(24 - tolerance, 104 + tolerance):
        for j in range(254 - tolerance, 344 + tolerance):
            if (i - 64) **2 + (j - 299)**2 <= (40+tolerance)**2:
                circle_map[i,j] = obstacle_color

    #Hexagon
    hexagon_map = np.zeros_like(my_map)       
    hexagon_pts= np.array([[164 - tolerance*1.2 ,170 + tolerance], 
                            [199,190+tolerance*1.6],    
                            [234 +tolerance,170+tolerance], 
                            [234 +tolerance,130 -tolerance], 
                            [199,110 - tolerance*1.2], 
                            [164 - tolerance,130 - tolerance]],float)             
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
    other_obstacle= np.array([[34 - (tolerance*2.5) ,64 - tolerance*0.6  ], 
                                [104 + tolerance*2,149 + (tolerance*4)], 
                                [89 + tolerance,69 + tolerance ], 
                                [114 + (tolerance*3),39 - (tolerance*(2))]],float)
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
    my_map[0:5,:]= [128,128,128]
    my_map[:,394:400]=[128,128,128]
    my_map[244:250,:]=[128,128,128]
    my_map[:,0:5] = [128,128,128]
    
    return my_map.copy()

import random

def testing_random_cases():
    global visited_map 
    global cost_map 
    global parent_map 
    for ite in range(100):
        start_x = random.randint(1,249)
        start_y = random.randint(1,399)
        goal_x =  random.randint(1,249)
        goal_y = random.randint(1,399)

        visited_map = np.zeros((250,400),dtype='uint16')
        cost_map = np.ones((250,400),dtype=float)*1000
        parent_map = np.zeros((250,400,2),dtype='uint16')
        my_map = np.zeros((250,400,3),dtype='uint8')
        my_map = np.zeros((250,400,3),dtype='uint8')
        obstacle_color = [255,255,255]
        tolerance_map = PopulateMap(my_map,obstacle_color,5)
        obstacle_map = PopulateMap(my_map,obstacle_color)
        my_map = cv.addWeighted(tolerance_map, 0.5, obstacle_map, 1, 0)
        start_node = nodes([start_x,start_y])
        start_node.cost = 0
        start_node.parent = [None,None]
        goal_node = nodes([goal_x,goal_y])
        cost_map[start_x,start_y] = 0.0
        visited_map[start_x,start_y] = 1
        parent_map[start_x,start_y] = [start_x,start_y]
        print("Start Node :",start_x,start_y, "Goal Node :",goal_x,goal_y)
        if check_validity_position(start_node,my_map):
            if check_validity_position(goal_node,my_map):
                DijkstraAlogrithm(start_node,goal_node,my_map)
            else:
                print("Invalid Goal Position")                
        else:
            print("Invalid Start Position")

def main():    
    #Tests 100 Random start and goal Nodes. 
    #COMMENT BELOW LINE FOR INDIVIDUAL TEST CASE
    ###########################################
    testing_random_cases()
    ###########################################

    #Define your START and GOAL positions here
    ##########################################
    start_x = 96
    start_y = 50
    goal_x = 8
    goal_y = 200
    ###########################################

    my_map = np.zeros((250,400,3),dtype='uint8')
    obstacle_color = [255,255,255]
    tolerance_map = PopulateMap(my_map,obstacle_color,5)
    obstacle_map = PopulateMap(my_map,obstacle_color)
    my_map = cv.addWeighted(tolerance_map, 0.5, obstacle_map, 1, 0)

    start_node = nodes([start_x,start_y])
    start_node.cost = 0
    start_node.parent = [None,None]
    goal_node = nodes([goal_x,goal_y])
    cost_map[start_x,start_y] = 0.0
    visited_map[start_x,start_y] = 1
    parent_map[start_x,start_y] = [start_x,start_y]

    if check_validity_position(start_node,my_map):
        if check_validity_position(goal_node,my_map):
            DijkstraAlogrithm(start_node,goal_node,my_map)
        else:
            print("Incorrect Nodes")
    else:
        print("Incorrect Nodes")
    
    
if __name__ == '__main__':
    main()
