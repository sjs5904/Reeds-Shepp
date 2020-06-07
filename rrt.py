import math
import random

import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, pi

class RRT:

    class Node:

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class Edge:

        def __init__(self, fnode, tnode):
            self.fromx = fnode.x
            self.fromy = fnode.y
            self.tox = tnode.x
            self.toy = tnode.y
            self.from_node = fnode
            self.to_node = tnode

    def __init__(self, start, goal, obstacle_list, rand_area,
                 expand_dis=0.5, path_resolution=0.5, goal_sample_rate=5, max_iter=500):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_randx = rand_area[0]
        self.max_randx = rand_area[1]
        self.min_randy = rand_area[2]
        self.max_randy = rand_area[3]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.edge_list = []

    def planning(self):

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_node_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_node_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if i % 5:
                self.draw_graph(rnd_node)

        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        # Euclidean norm
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) >= self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_randx, self.max_randx),
                            random.uniform(self.min_randy, self.max_randy))
        else:
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-3, 3, -1, 1])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def check_collision(node, obstacleList, area):
        
        if node is None:
            return False

        if node.x < area[0] or node.x > area[1] or node.y < area[2] or node.y > area[3]:
            return False
        
        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False

        return True 

    @staticmethod
    def plot_circle(x, y, size, color="-k"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    def getAngle(self, a, b, c):
        ang = math.degrees(abs(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0])))
        if ang < 0:
            ang = ang + 180

        if ang > 180:
            ang = ang - 180
            
        return ang

    def get_nearest_point_on_edge(self, edge, node):
        angle = (edge.toy - edge.fromy) / (edge.tox - edge.fromx)
        a = edge.fromy - edge.toy
        b = -(edge.fromx - edge.tox)
        c = edge.fromx*edge.toy + edge.tox*edge.fromy
        if self.getAngle((edge.fromx,edge.fromy),(edge.tox,edge.toy),(node.x,node.y)) < 90 and self.getAngle((edge.tox,edge.toy),(edge.fromx,edge.fromy),(node.x,node.y)) < 90:
            temp = -1 * (a * node.x + b * node.y + c) / (a * a + b * b)
            x = temp * a + node.x
            y = temp * b + node.y
            new_node = self.Node(x, y)
            new_node.path_x = edge.from_node.path_x
            new_node.path_y = edge.from_node.path_y
            new_node.parent = edge.from_node
            edge.to_node.parent = new_node
            return new_node
        elif math.hypot((edge.fromx - node.x),(edge.fromy - node.y)) > math.hypot((edge.tox - node.x),(edge.toy - node.y)):
            return edge.to_node
        return edge.from_node

    
    def get_edge_dist(self, edge, node):
        a = edge.fromy - edge.toy
        b = -(edge.fromx - edge.tox)
        c = edge.fromx*edge.toy + edge.tox*edge.fromy
        if self.getAngle((edge.fromx,edge.fromy),(edge.tox,edge.toy),(node.x,node.y)) < 90 and self.getAngle((edge.tox,edge.toy),(edge.fromx,edge.fromy),(node.x,node.y)) < 90:
            return abs(a*node.x+b*node.y+c)/math.hypot(a,b)
        return min(math.hypot(node.x-edge.fromx,node.y-edge.fromy), math.hypot(node.x-edge.tox,node.y-edge.toy))
            

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [math.sqrt((node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2) for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    
    def get_nearest_edge_index(self, edge_list, rnd_node):
        dlist = [self.get_edge_dist(edge, rnd_node) for edge in edge_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_node_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False  # collision

        return True  
    
    def check_edge_collision(self, edge, obstacleList):

        if edge is None:
            return False

        for (ox, oy, size) in obstacleList:
            if self.get_edge_dist(edge, self.Node(ox,oy)) < size:
                return False  # collision

        return True  

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def plot_circle(x, y, size, color="-k"):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)
    
def main():
    sx=-2.0
    sy=-0.5
    gx=2.0
    gy=-0.5
    
    obstacleList = [
        (0, 1, 0.98),
        (0, -1, 0.98)
    ]
    
    rrt = RRT(start=[sx, sy],
              goal=[gx, gy],
              rand_area=[-3, 3, -1, 1],
              obstacle_list=obstacleList)
    path = rrt.planning()

    if path is None:
        print("Cannot find path")
    else:
        print("found path")
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01) 
        plt.show()


if __name__ == '__main__':
    main()
