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
        self.parent = [0,0]

visited_map = np.zeros((252,402))
cost_map = np.ones((252,402))*1000
class lists():
    def __init__(self):
        self.OpenNodes = []
        self.ClosedNodes = []
        self.visited = []
class action():
    def __init__(self):
        self.action_sets= [(1,0),(-1,0), (0,1), (0,-1), (1,1), (-1,1),(1,-1),(-1,-1)]
        self.cost = [1,1,1,1,1.4,1.4,1.4,1.4]

def DijkstraAlogrithm(start,goal,my_map):
    current = start
    my_lists = lists()
    # print(len(my_lists.OpenNodes))
    my_lists.visited.append(current)
    my_lists.OpenNodes.append(current)
    path_map = my_map.copy()
    
    count = 0
    while current.position != goal.position :
        check_future_state(current,my_map,my_lists,path_map)
        current = my_lists.OpenNodes[0]
        
        cv.imshow("Path",path_map)
        cv.waitKey(1)
        count = count +1
        print(count)
        print(current.position)
    print("Goal reached",current.position)
    # for i in range(len(my_lists.visited)-1):
    #     print(my_lists.visited[i].parent)

def check_validity_position(node,my_map):
    position = node.position
    # print(position[0],position[1])
    if (my_map[position[0],position[1],0]==255):
        # print("Invalid Start or Goal node")
        return False
    else:
        # print("Valid Goal and Start Nodes")
        return True

def hashmap_implementation(future):
    if visited_map[future.position[0],future.position[1]]==1:
        if cost_map[future.position[0],future.position[1]] > future.cost:
            cost_map[future.position[0],future.position[1]] = future.cost
        return False
    else:
        visited_map[future.position[0],future.position[1]] = 1
        return True



def check_visited(visited,future):
    # print(len(visited))
    for ite in range((len(visited)-1),-1,-1):
        # print(ite)
        if ((visited[ite].position[0]==future.position[0])and(visited[ite].position[1]==future.position[1])):
            # print("Already visited")
            if visited[ite].cost>future.cost:
                visited[ite].cost = future.cost
                visited[ite].parent = future.parent
            return False
    
    visited.append(future)
            # print("not visited")
    return True
    # print("Called Once")
    exit()

def check_future_state(current,my_map,my_lists,path_map):
    actions = action()
    lowest_cost = np.inf
    temp_nodes = []
    
    for ite in range(8):
        move = [0,0]
        move[0] = current.position[0] + actions.action_sets[ite][0]
        move[1] = current.position[1] + actions.action_sets[ite][1]
        cost = 0
        cost = current.cost + actions.cost[ite]
        # cost +=  actions.cost[ite]
        
        if (int(my_map[move[0],move[1],1])!=255):
            
            future = nodes([0,0])
            future.position = move
            future.cost = cost
            future.parent = current.position
            if hashmap_implementation(future):
            # if (check_visited(my_lists.visited,future)):
                my_lists.OpenNodes.append(future)
                # endix = len(my_lists.OpenNodes)-1
                if (int(my_map[move[0],move[1],1])!=255):
                    path_map[my_lists.OpenNodes[0].position[0],my_lists.OpenNodes[0].position[1]]=[0,255,0]
                # my_map[future.position,:] = [255,0,0]
                # cv.imshow("my_map",my_map)
                # cv.waitKey(1)
                # print("This print works",my_map[move[0],move[1],0])
                # print("Temp node length :",len(temp_nodes))
        else:
            print("Obstacle Detected")
    # temp_nodes= sort_array_by_cost(temp_nodes)
    # for i in range(len(temp_nodes)):
    #     my_lists.OpenNodes.append(temp_nodes.pop(0))
    my_lists.OpenNodes.pop(0) 
    # print(len(my_lists.OpenNodes))
def sort_array_by_cost(nodes):
    # temp = []
    for i in range(len(nodes)-1):
        for j in range(i):
            if nodes[i].cost > nodes[j].cost:
                temp = nodes[j]
                nodes[j] = nodes[i]
                nodes[i] = temp

    return nodes

def BackTrace():
    return 0

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
    # side2 = side2 * side1
    side2 = cv.bitwise_and(side2,side1)
    side3=half_planes(hexagon_map,hexagon_pts[2],hexagon_pts[3],obstacle_color,True)
    side4=half_planes(hexagon_map,hexagon_pts[3],hexagon_pts[4],obstacle_color,True)
    side5=half_planes(hexagon_map,hexagon_pts[4],hexagon_pts[5],obstacle_color,False)
    # side5=side5*side4
    side5=cv.bitwise_and(side5,side4)
    side6=half_planes(hexagon_map,hexagon_pts[5],hexagon_pts[0],obstacle_color,False)
    # side6=side6*side3
    side6 = cv.bitwise_and(side3,side6)
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
    # side2 = side1*side2
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
    # my_map = my_map.astype(int)
    return my_map

def main():
    my_map = np.zeros((252,402,3),dtype='uint8')
    obstacle_color = [255,255,255]
    my_map = PopulateMap(my_map,obstacle_color)
    print(my_map.shape)
    # print("Hex codrs", my_map[164,170])
    # cv.imshow("my map ",my_map)
    # cv.waitKey(0)
    start_node = nodes([2,2])
    start_node.cost = 0
    start_node.parent = [0,0]
    goal_node = nodes([132,112])
    
    if check_validity_position(start_node,my_map):
        if check_validity_position(goal_node,my_map):
            DijkstraAlogrithm(start_node,goal_node,my_map)

    
if __name__ == '__main__':
    main()
