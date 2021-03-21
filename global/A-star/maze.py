#!/usr/bin/env python3

import numpy as np
from heapq import heappop, heappush
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Maze:
    def __init__(self):
        self.min_x = 0
        self.min_y = 0
        self.max_x = 10
        self.max_y = 10
        self.start = np.array([0, 0])
        self.goal = np.array([5, 10])
        self.directions = np.array([
            [1, 0], [-1, 0], [0, 1], [0, -1],
            [1, 1], [-1, 1], [-1, 1], [-1, -1],
            ])

        self.fig, self.ax = plt.subplots(1,1)
        self.ax.set_aspect('equal')
        self.setWall()

    def get_start_loc(self):
        return self.start

    def get_goal_loc(self):
        return self.goal

    def is_goal(self, loc):
        return all(loc == self.goal)

    def setWall(self):
        self.outer_walls = np.array([
                [self.min_x-1, self.min_y-1],
                [self.min_x-1, self.max_y+1],
                [self.max_x+1, self.max_y+1],
                [self.max_x+1, self.min_y-1],
                [self.min_x-1, self.min_y-1],
                ])

        #self.inner_walls = np.array([
        #        [1, 1],
        #        [1, 3],
        #        [2, 3],
        #        [3, 4],
        #        [2, 4],
        #        [4, 5],
        #        [2, 1],
        #        [5, 1],
        #        ])

        self.inner_walls = np.random.randint(self.min_x, self.max_x, (40, 2)) 
        self.inner_walls[0] = np.array([0,0])
        for idx in range(len(self.inner_walls)):
            if all(self.inner_walls[idx] == self.start):
                self.inner_walls[idx] += np.random.randint(self.min_x, self.max_x, 2)
            elif all(self.inner_walls[idx] == self.goal):
                self.inner_walls[idx] -= np.random.randint(self.min_x, self.max_x, 2)
 

    def is_valid_area(self, loc):
        if self.min_x <= loc[0] <= self.max_x and self.min_y <= loc[1] <= self.max_y:
            for wall in self.inner_walls:
                if all(wall == loc):
                    return False

            return True
        else:
            return False

    def movable_coord(self, loc):
        destination = []
        for direction in self.directions:
            tmp = loc + direction
            if self.is_valid_area(tmp):
                destination.append(tmp)

        return destination

            

    def drawMaze(self, route=None):
        self.ax.plot(self.outer_walls[:,0], self.outer_walls[:,1], color='k', lw=2)
        for inner_wall in self.inner_walls:
            self.ax.scatter(inner_wall[0], inner_wall[1], color='k', s=200, marker='s')

        start_marker = patches.Circle(xy=self.start, radius=0.4, fc='red', fill=True)
        goal_marker  = patches.Circle(xy=self.goal,  radius=0.4, fc='blue', fill=True)
        self.ax.add_patch(start_marker)
        self.ax.add_patch(goal_marker)

        if route != None:
            for item in route:
                marker = patches.Circle(xy=item, radius=0.1, fc='red', fill=True)
                self.ax.add_patch(marker)

        plt.show()

class Node:
    def __init__(self, loc, g_cost, h_cost, parent):
        self.location = loc # ndarray
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent_node = parent

    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        self_cost = self.f_cost()
        other_cost = other.f_cost()
        
        if self_cost == other_cost:
            result = self.g_cost < other.g_cost
        else:
            result = self_cost < other_cost

        return result

    def __eq__(self, other):
        result = all(self.location == other.location)
        return result

def main():
    maze = Maze()
    start_loc = maze.get_start_loc()
    goal_loc = maze.get_goal_loc()
    start_node = Node(loc=start_loc, g_cost=0, h_cost=np.linalg.norm(goal_loc - start_loc), parent=None)
    done = False

    open_nodes = []
    close_nodes = []

    heappush(open_nodes, start_node)
    #print('open_nodes', len(open_nodes))
    #for item in open_nodes:
    #    print(item.location)

    while True:
        if len(open_nodes) == 0:
            print('There is no complete path !')
            break
        
        #print('*'*100)
        base_node = heappop(open_nodes)

        close_nodes.append(base_node)

        #print('base_node', base_node.location)
        
        #移動可能ノードを地図から取得
        movable_locations = maze.movable_coord(base_node.location)
        #print('movable_locations', movable_locations)

        for movable_loc in movable_locations:
            #print('*'*100)
            new_node = Node(loc=movable_loc, g_cost=base_node.g_cost+1, h_cost=np.linalg.norm(goal_loc - movable_loc), parent=base_node.location)
            #print(new_node.location, new_node.f_cost())

            if (new_node not in open_nodes) and (new_node not in close_nodes):
                heappush(open_nodes, new_node)

            elif new_node in open_nodes:
                for opened_node in open_nodes:
                    if new_node == opened_node and new_node.f_cost() < opened_node.f_cost():
                        open_nodes.remove(opened_node)
                        heappush(open_nodes, new_node)
                        break

            elif new_node in close_nodes:
                for closed_node in close_nodes:
                    if new_node == closed_node and new_node.f_cost() < closed_node.f_cost():
                        close_nodes.remove(closed_node)
                        heappush(open_nodes, new_node)
                        break
        #print('open_nodes', len(open_nodes))
        #for item in open_nodes:
        #    print(item.location)
        #print('clsoe_nodes', len(close_nodes))
        #for item in close_nodes:
        #    print(item.location)

        #base_node がゴールならばbreak
        if maze.is_goal(base_node.location):
            done = True
            print('goal')
            break

    #print('close_nodes')
    #for item in close_nodes:
    #    print(item.location, '<-', item.parent_node)

    route = []
    if done:
        print('ROUTE')
        dstn_loc = goal_loc
        route.insert(0, dstn_loc)
        while True:
            for node in close_nodes:
                if all(node.location == dstn_loc):
                    dstn_loc = node.parent_node
                    route.insert(0, dstn_loc)
            if all(dstn_loc == start_loc):
                break

        print(route)
    maze.drawMaze(route)


if __name__ == '__main__':
    main()
