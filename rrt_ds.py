from scipy.spatial import KDTree
import numpy as np
import vision
import random
import math
import time

class node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRT(object):
    def __init__(self, N_sample=1000):
        self.N_sample = N_sample
        self.threshold = 300
        self.min_x = -6000
        self.max_x = 6000
        self.min_y = -4500
        self.max_y = 4500
        self.expand_length = 300
        self.robot_size = 200
        self.avoid_distance = 200
        self.obstacle_x = []
        self.obstacle_y = []

    def plan(self, vision, start_x, start_y, goal_x, goal_y):
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                self.obstacle_x.append(robot_blue.x)
                self.obstacle_y.append(robot_blue.y)

        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                self.obstacle_x.append(robot_yellow.x)
                self.obstacle_y.append(robot_yellow.y)

       #print(self.obstacle_x)
        # start_node = node(start_x, start_y)
        node_list_x = [start_x]
        node_list_y = [start_y]
        node_list_parent = [None]
        # node_list = [start_node]

        for i in range(self.N_sample):
            if i == self.N_sample - 1:
                flag = 0
            rand = self.random_sampling()

            min_index = self.find_nearest_node(node_list_x, node_list_y, rand)
            nearest_node_x = node_list_x[min_index]
            nearest_node_y = node_list_y[min_index]

            theta = math.atan2(rand.y-nearest_node_y, rand.x-nearest_node_x)

            new_x = nearest_node_x + self.expand_length * math.cos(theta)
            new_y = nearest_node_y + self.expand_length * math.sin(theta)
            # new_node = node(new_x, new_y, min_index)

            if self.check_ob(new_x, new_y):
                continue

            node_list_x.append(new_x)
            node_list_y.append(new_y)
            node_list_parent.append(min_index)

            if self.distance(new_x, new_y, goal_x, goal_y) < self.threshold:
                print('Path Found!')
                print('Iterations:', i)
                flag = 1
                break
        path_x = []
        path_y = []
        index = len(node_list_x) - 1

        start3 = time.time()
        while node_list_parent[index] is not None:
            path_x.append(node_list_x[index])
            path_y.append(node_list_y[index])
            index = node_list_parent[index]


        path_x.append(start_x)
        path_y.append(start_y)

        path_x.reverse()
        path_y.reverse()

        end3 = time.time()
        #print("3  ", end3-start3)

        print('End!')
        return path_x, path_y, flag


    def random_sampling(self):
        rand_x = random.uniform(self.min_x, self.max_x)
        rand_y = random.uniform(self.min_y, self.max_y)
        return node(rand_x, rand_y)


    def distance(self, point1_x, point1_y, point2_x, point2_y):
        return math.hypot(point1_x - point2_x, point1_y - point2_y)

    # def find_nearest_node(self, node_list_x, node_list_y, node):
    #     dist =[]
    #     start1 = time.time()
    #     node_list_x = numpy.array(node_list_x)
    #     node_list_y = numpy.array(node_list_y)

    #     dist = (node_list_x - node.x) ** 2 + (node_list_y - node.y) ** 2
        
    #     min_index = dist.index(min(dist))
    #     end2 = time.time()
    #     #print("2  ", end2-start2)
    #     return min_index

    def find_nearest_node(self, x, y, node):
        tree = KDTree(np.vstack((x, y)).T)
        distance, index = tree.query(np.array([node.x, node.y]), eps=0.5)
        return index

    def check_ob(self, new_node_x, new_node_y):
        dist = []
        # for xx, yy in self.obstacle_x, self.obstacle_y:
        #     dist.append(self.distance(nd.x, nd.y, xx, yy))
        for i in range(len(self.obstacle_x)):
            dist.append(self.distance(new_node_x, new_node_y, self.obstacle_x[i], self.obstacle_y[i]))
        for d in dist:
            if d < self.robot_size + self.avoid_distance:
                return True

        return False








