#!/usr/bin/env python3
import copy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
from tqdm import tqdm

class WorldMap:
    def __init__(self):
        self.min_x = -5
        self.max_x =  5
        self.min_y = -5
        self.max_y =  5

        self.obstacles = []
        #self.setObstacles(np.array([1.5, 1.5]), 1.5)
        #self.setObstacles(np.array([3.0, -1.5]), 1.0)
        #self.setObstacles(np.array([-2.0, 2.5]), 1.5)
        #self.setObstacles(np.array([0.0, -1.5]), 1.0)

    def isValidArea(self, loc):
        if (self.min_x < loc[0] < self.max_x) and (self.min_y < loc[1] < self.max_y):
            for obs in self.obstacles:
                if np.linalg.norm(loc-obs['center']) < obs['rad']:
                    return False
            return True
        else:
            return False

    def setObstacles(self, center, rad):
        circle = {'center':center, 'rad':rad}
        self.obstacles.append(circle)

    def getObstacles(self):
        return self.obstacles

class Node():
    def __init__(self, loc):
        self.loc = loc
        self.cost = 0.0
        self.parent = None

    def printInfo(self):
        print('** Node Info **')
        print('loc      :', self.loc)
        print('cost     :', self.cost)
        print('parent ID:', self.parent)
        print('***************')

class RRTstar:
    def __init__(self, map_, start_loc, goal_loc):
        # Hyper Parameters
        self.dl = 0.2
        self.goal_sampling_rate = 0.3

        # Map Information
        self.map = map_
        self.minX = self.map.min_x
        self.maxX = self.map.max_x
        self.minY = self.map.min_y
        self.maxY = self.map.max_y
        
        self.start_node = Node(start_loc)
        self.goal_node  = Node(goal_loc)
        self.node_list = []

    def solve(self):
        self.node_list.append(self.start_node)
        #print('start_node')
        #print(self.node_list[0].loc[0])
        #print(self.node_list[0].loc[1])
        target_point = self.targetPoint()
        #print('target_node')
        #print(target_point)
        nearest_node_id = self.getNearestNodeIndex(target_point)
        new_node = self.makeNewNode(target_point, nearest_node_id)

        if self.map.isValidArea(new_node.loc):
            print('Valid')

    def makeNewNode(self, target_point, nearest_node_id):
        nearest_node = self.node_list[nearest_node_id]
        #print(nearest_node.loc)
        new_node = copy.deepcopy(nearest_node) #copy.copy()だとnew_nodeを変更するとnearest_nodeも変わってしまう 
        unit_vec = (target_point - nearest_node.loc) / np.linalg.norm(target_point - nearest_node.loc)
        new_node.loc += self.dl * unit_vec
        new_node.cost += self.dl
        new_node.parent = nearest_node_id

        new_node.printInfo()
        return new_node

    def getNearestNodeIndex(self, target_point):
        distance_list = [np.linalg.norm(node.loc - target_point)
                for node in self.node_list]
        #print(distance_list)
        return np.argmin(distance_list)

    def targetPoint(self):
        if np.random.rand() < self.goal_sampling_rate:
            return self.goal_node.loc
        else:
            return np.array([
                (self.maxX - self.minX) * np.random.rand() + self.minX,
                (self.maxY - self.minY) * np.random.rand() + self.minY])
    
def main():
   world_map = WorldMap()
   start_location = np.array([-2.0, 2.0])
   goal_location  = np.array([4.0, 4.0])
   planner = RRTstar(world_map, start_loc=start_location, goal_loc=goal_location)
   planner.solve()


if __name__ == '__main__':
    main()
