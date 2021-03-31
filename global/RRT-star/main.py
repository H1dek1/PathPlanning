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
        for i in range(100):
            target_point = self.targetPoint()
            #print('target_node')
            #print(target_point)
            nearest_node_id = self.getNearestNodeIndex(target_point)
            new_node = self.makeNewNode(target_point, nearest_node_id)
            if self.map.isValidArea(new_node.loc):
                print('Valid')
                near_node_ids = self.findNearNodes(new_node)
                new_node = self.chooseParent(new_node, near_node_ids)
                self.node_list.append(new_node)
                self.connectNodes(new_node, near_node_ids)

    def connectNodes(self, new_node, near_ids):
        for near_id in near_ids:
            near_node = self.node_list[near_id]
            print('near_node.loc', near_node.loc)

            relative_vector = near_node.loc - new_node.loc
            length = np.linalg.norm(relative_vector)
            angle = np.arctan2(relative_vector[1], relative_vector[0])

            tmp_cost = new_node.cost + length
            print('new_node.cost', new_node.cost)
            print('tmp_cost', tmp_cost)

            if (near_node.cost > tmp_cost) and (self.isValidEdge(near_node, length, angle)):
                    near_node.parent = len(self.node_list) - 1
                    near_node.cost = tmp_cost

    def chooseParent(self, new_node, near_ids):
        if len(near_ids) == 0:
            new_node

        distance_list = []
        for near_id in near_ids:

            relative_vector = self.node_list[near_id].loc - new_node.loc
            length = np.linalg.norm(relative_vector)
            angle = np.arctan2(relative_vector[1], relative_vector[0])

            if self.isValidEdge(self.node_list[near_id], length, angle):
                distance_list.append(self.node_list[near_id].cost + length)

            else:
                distance_list.append(np.inf)

        print('distance_list', distance_list)
        minimum_cost = min(distance_list)
        minimum_id   = near_ids[distance_list.index(minimum_cost)]
        #if minimum_cost == np.inf:
        #    return new_node
        #else:
        #    new_node.cost = minimum_cost
        #    new_node.parent = minimum_id
        #    print(new_node.cost)
        #    print(new_node.parent)
        #    return new_node

        if minimum_cost != np.inf:
            # 絶対ここを通るはず
            new_node.cost = minimum_cost
            new_node.parent = minimum_id
            print(new_node.cost)
            print(new_node.parent)

        return new_node


    def isValidEdge(self, start_node, length, angle):
        tmp_node = copy.deepcopy(start_node)

        for i in range(int(length / self.dl)):
            tmp_node.loc += self.dl * np.array([np.cos(angle), np.sin(angle)])
            if not self.map.isValidArea(tmp_node.loc):
                return False

        return True

    def findNearNodes(self, center_node):
        n_nodes = len(self.node_list) + 2
        search_rad = 10*(self.maxX - self.minX) * np.sqrt(np.log(n_nodes)/n_nodes)
        #search_rad = 5.0 * self.dl
        print(search_rad)
        distance_list = [np.linalg.norm(node.loc - center_node.loc)
                for node in self.node_list]
        near_ids = [distance_list.index(i) for i in distance_list if i <= search_rad]
        print(near_ids)
        return near_ids

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

    def render(self):
        print('node_list', len(self.node_list))
        fig, ax = plt.subplots(1,1)
        def update(i):
            if i%10 == 0: print(i, '/ {}'.format(100+3))
            if i != 0:
                plt.cla()

            ax.set_xlim(self.minX, self.maxX)
            ax.set_ylim(self.minY, self.maxY)
            ax.set_xlabel(r'$x$')
            ax.set_ylabel(r'$y$')
            ax.set_aspect('equal')

            for j in range(i):
                node_0 = self.node_list[j].loc
                if self.node_list[j].parent == None:
                    continue
                node_1 = self.node_list[self.node_list[j].parent].loc

                x = np.array([node_0[0], node_1[0]])
                y = np.array([node_0[1], node_1[1]])
                ax.plot(x, y, marker='.', color='k')
            ax.scatter(self.start_node.loc[0], self.start_node.loc[1], marker='o', color='r')
            ax.scatter(self.goal_node.loc[0], self.goal_node.loc[1], marker='o', color='b')

        ani = animation.FuncAnimation(fig, update, interval=400, frames=100)
        ani.save('anim.mp4', writer='ffmpeg')
    
def main():
   world_map = WorldMap()
   start_location = np.array([-2.0, -2.0])
   goal_location  = np.array([4.0, 4.0])
   planner = RRTstar(world_map, start_loc=start_location, goal_loc=goal_location)
   planner.solve()
   planner.render()


if __name__ == '__main__':
    main()
