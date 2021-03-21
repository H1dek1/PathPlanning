#!/usr/bin/env python3
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
        self.setObstacles(np.array([1.5, 1.5]), 1.5)
        self.setObstacles(np.array([3.0, -1.5]), 1.0)
        self.setObstacles(np.array([-2.0, 2.5]), 1.5)
        self.setObstacles(np.array([0.0, -1.5]), 1.0)

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

    def isInObstacle(self, loc):
        pass


class RRT:
    def __init__(self, map_, start_loc, goal_loc):
        self.dl = 0.2

        self.map = map_
        self.minX = self.map.min_x
        self.maxX = self.map.max_x
        self.minY = self.map.min_y
        self.maxY = self.map.max_y

        self.start_loc = start_loc
        self.goal_loc  = goal_loc
        self.node_list = np.empty((0,2), dtype=float)
        self.edge_list = []
        self.target_list = np.empty((0,2), dtype=float)
        self.node_list = np.append(self.node_list, np.array([self.start_loc]), axis=0)

    def targetPoint(self):
        epsilon = 0.3
        if np.random.rand() < epsilon:
            return self.goal_loc
        else:
            return np.array([
            (self.maxX - self.minX) * np.random.rand() + self.minX,
            (self.maxY - self.minY) * np.random.rand() + self.minY,
            ])

    def solve(self):
        is_clear = False
        self.n_steps = 0
        for step in range(1000):

            while True:
                target_point = self.targetPoint()
                distance_list = np.array([np.linalg.norm(item) for item in (target_point - self.node_list)])
                nearest_id = distance_list.argmin()

                unit_vec = (target_point - self.node_list[nearest_id]) / np.linalg.norm(target_point - self.node_list[nearest_id])
                new_node = self.node_list[nearest_id] + self.dl*unit_vec
                if self.map.isValidArea(new_node):
                    break

            self.target_list = np.append(self.target_list, np.array([target_point]), axis=0)

            self.node_list = np.append(self.node_list, np.array([new_node]), axis=0)
            self.edge_list.append(([nearest_id, len(self.node_list)-1]))
            if self.isGoal(new_node):
                self.target_list = np.append(self.target_list, np.array([self.goal_loc]), axis=0)
                self.node_list = np.append(self.node_list, np.array([self.goal_loc]), axis=0)
                self.edge_list.append(([len(self.node_list)-2, len(self.node_list)-1]))
                is_clear = True
                self.n_steps = step
                break

        if is_clear:
            print('Success')
        else:
            print('Failed')
            #print(target_point)
            #print(nearest_id)
        #print('edge_list\n', self.edge_list)
        #print('node_list\n', self.node_list)


    def isGoal(self, node):
        return np.linalg.norm(self.goal_loc - node) < self.dl

    def render(self):
        print('node_list', len(self.node_list))
        fig, ax = plt.subplots(1,1)
        def update(i):
            if i%10 == 0: print(i, '/ {}'.format(self.n_steps+3))
            if i != 0:
                plt.cla()

            ax.set_xlim(self.minX, self.maxX)
            ax.set_ylim(self.minY, self.maxY)
            ax.set_xlabel(r'$x$')
            ax.set_ylabel(r'$y$')
            ax.set_aspect('equal')

            for j in range(i):
                x = np.array([self.node_list[self.edge_list[j][0]][0], self.node_list[self.edge_list[j][1]][0]])
                y = np.array([self.node_list[self.edge_list[j][0]][1], self.node_list[self.edge_list[j][1]][1]])
                if i == self.n_steps+2:
                    ax.plot(x, y, marker='.', color='k')
                else:
                    ax.plot(x, y, marker='.', color='k')


            ax.scatter(self.start_loc[0], self.start_loc[1], marker='o', color='r')
            ax.scatter(self.goal_loc[0], self.goal_loc[1], marker='o', color='b')
            if i > 0:
                ax.scatter(self.target_list[i-1][0], self.target_list[i-1][1], marker='o', color='g')
            obstacles = self.map.getObstacles()
            for obs in obstacles:
                circle = patches.Circle(xy=obs['center'], radius=obs['rad'], ec='k', fill=False)
                ax.add_patch(circle)


        ani = animation.FuncAnimation(fig, update, interval=400, frames=self.n_steps+3)
        ani.save('anim.mp4', writer='ffmpeg')




def main():
    world_map = WorldMap()
    start_location = np.array([-2.0, -2.0])
    goal_location  = np.array([4., 4.])
    planner = RRT(world_map, start_loc=start_location, goal_loc=goal_location)
    planner.solve()

    planner.render()

if __name__ == '__main__':
    main()
