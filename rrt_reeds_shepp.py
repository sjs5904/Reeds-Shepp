import copy
import math
import os
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, pi

try:
    import reeds_shepp_planner
    from rrt import RRT
except ImportError:
    raise


class RRTReedsShepp(RRT):
    
    class Node(RRT.Node):

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter):
        
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.area = rand_area
        self.min_x = rand_area[0]
        self.max_x = rand_area[1]
        self.min_y = rand_area[2]
        self.max_y = rand_area[3]
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_failed_list = []

        self.min_radius = 0.4
        self.step_size = 0.02
        self.goal_dist = 0.5
        self.goal_sample_rate = 0.01

    def planning(self):

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(new_node, self.obstacle_list, self.area):
                self.node_list.append(new_node)
            else:
                self.node_failed_list.append(new_node)

            if i % 5 == 0:
                self.plot_start_goal_arrow()
                self.draw_graph(rnd)

            if new_node and random.randint(0, 100) >= self.goal_sample_rate:
                last_index = self.search_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("Maximum Iteration reached")

        last_index = self.search_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None
    
    def try_goal_path(self, node):

        goal = self.Node(self.end.x, self.end.y, self.end.yaw)

        new_node = self.steer(node, goal)
        if new_node is None:
            return

        if self.check_collision(new_node, self.obstacle_list, self.area):
            self.node_list.append(new_node)
    
    def steer(self, from_node, to_node):

        px, py, pyaw, mode, clengths = reeds_shepp_planner.reeds_shepp_planner(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.min_radius, self.step_size)

        if not px:
            return None

        for i in range(0, len(px)):
            if px[i]<self.min_x or px[i]>self.max_x or py[i]<self.min_y or py[i]>self.max_y:
                return None
        
        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.parent = from_node
        
        return new_node

    def get_random_node(self):

        rnd = self.Node(random.uniform(self.min_x, self.max_x),
                        random.uniform(self.min_y, self.max_y),
                        random.uniform(-math.pi, math.pi)
                        )
        
        return rnd

    def search_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_dist:
                goal_indexes.append(i)
        
        if not goal_indexes:
            return None
        
        for i in goal_indexes:
            nod = self.steer(self.node_list[i], self.end)
            if self.check_collision(nod, self.obstacle_list, self.area):
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        
        return path

    def draw_graph(self, rnd=None):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            reeds_shepp_planner.plot_arrow(
                node.x, node.y, node.yaw, length=0.01, width=0.15, fc="g", ec="k")
            if node.parent:
                plt.plot(node.path_x, node.path_y, "r")

        for node in self.node_failed_list:
            if node:
                reeds_shepp_planner.plot_arrow(
                    node.x, node.y, node.yaw, length=0.01, width=0.15, fc="k", ec="k")

        for (x, y, r) in self.obstacle_list:
            self.plot_circle(x, y, r)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-3.1, 3.1, -1.05, 1.05])
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.001)

    def plot_start_goal_arrow(self):
        reeds_shepp_planner.plot_arrow(
            self.start.x, self.start.y, self.start.yaw, length=0.01, width=0.2, fc="r", ec="k")
        reeds_shepp_planner.plot_arrow(
            self.end.x, self.end.y, self.end.yaw, length=0.01, width=0.2, fc="b", ec="k")
        
    def plot_circle(self, x, y, size, color="-k"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)


def main():

    dt = 0.2
    
    obstacleList = [
        (0, 1, 1.0 - dt),
        (0, -1, 1.0 - dt)
    ]
    
    start = [-2.0, -0.5, 0.0]
    goal = [2.0, -0.5, math.pi / 2]

    rrt_reeds_shepp = RRTReedsShepp(start, goal, obstacleList, [-3, 3, -1, 1], 400)
    path = rrt_reeds_shepp.planning()

    if path:
        rrt_reeds_shepp.draw_graph()
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path], '-b')
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()
