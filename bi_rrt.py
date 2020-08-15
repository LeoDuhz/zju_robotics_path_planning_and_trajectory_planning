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

class Bi_RRT(object):
    def __init__(self, N_sample=500):
        self.N_sample = N_sample
        self.threshold = 200
        self.min_x = -6000
        self.max_x = 6000
        self.min_y = -4500
        self.max_y = 4500
        self.expand_length = 150
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
        start_node = node(start_x, start_y)
        node_list1 = [start_node]

        end_node = node(goal_x, goal_y)
        node_list2 = [end_node]

        for i in range(self.N_sample):
            # print(i)
            rand = self.random_sampling()

            min_index1 = self.find_nearest_node(node_list1, rand)
            nearest_node1 = node_list1[min_index1]

            theta1 = math.atan2(rand.y-nearest_node1.y, rand.x-nearest_node1.x)

            new_x1 = nearest_node1.x + self.expand_length * math.cos(theta1)
            new_y1 = nearest_node1.y + self.expand_length * math.sin(theta1)
            new_node1 = node(new_x1, new_y1, min_index1)

            if not self.check_ob(new_node1, nearest_node1):
                node_list1.append(new_node1)

                min_index2 = self.find_nearest_node(node_list2, new_node1)
                nearest_node2 = node_list2[min_index2]

                theta2 = math.atan2(new_node1.y-nearest_node2.y, new_node1.x-nearest_node2.x)
                new_x2 = nearest_node2.x + self.expand_length * math.cos(theta2)
                new_y2 = nearest_node2.y + self.expand_length * math.sin(theta2)

                new_node2 = node(new_x2, new_y2, min_index2)

                if not self.check_ob(new_node2, nearest_node2):
                    node_list2.append(new_node2)
                    index = len(node_list2)-1

                    while self.distance(new_node1.x, new_node1.y, new_node2.x, new_node2.y) > self.threshold:
                        theta3 = math.atan2(new_node1.y - new_node2.y, new_node1.x - new_node2.x)
                        new_x3 = new_node2.x + self.expand_length * math.cos(theta3)
                        new_y3 = new_node2.y + self.expand_length * math.sin(theta3)

                        new_node3 =node(new_x3, new_y3, len(node_list2)-1)
                        if not self.check_ob(new_node3, nearest_node2):
                            node_list2.append(new_node3)
                            new_node2 = new_node3
                        else:
                            break
                    # print('Here!')
                if self.distance(new_node1.x, new_node1.y, new_node2.x, new_node2.y) < self.threshold:
                    print('Path Found!')
                    break
            if len(node_list1) > len(node_list2):
                tmp = node_list2
                node_list2 = node_list1
                node_list1 = tmp

        path_x1 = []
        path_y1 = []
        index1 = len(node_list1)-1
        while node_list1[index1].parent is not None:
            path_x1.append(node_list1[index1].x)
            path_y1.append(node_list1[index1].y)
            index1 = node_list1[index1].parent
        if path_x1[len(path_x1)-1] < -1000:
            path_x1.append(start_x)
            path_y1.append(start_y)
            path_x1.reverse()
            path_y1.reverse()
            path_x2 = []
            path_y2 = []
            index2 = len(node_list2) - 1
            while node_list2[index2].parent is not None:
                path_x2.append(node_list2[index2].x)
                path_y2.append(node_list2[index2].y)
                index2 = node_list2[index2].parent

            path_x2.append(goal_x)
            path_y2.append(goal_y)

            path_x = path_x1 + path_x2
            path_y = path_y1 + path_y2

            return path_x, path_y

        elif path_x1[len(path_x1)-1] > 1000:
            path_x1.append(goal_x)
            path_y1.append(goal_y)

            path_x2 = []
            path_y2 = []
            index2 = len(node_list2) - 1
            while node_list2[index2].parent is not None:
                path_x2.append(node_list2[index2].x)
                path_y2.append(node_list2[index2].y)
                index2 = node_list2[index2].parent
            path_x2.append(start_x)
            path_y2.append(start_y)
            path_x2.reverse()
            path_y2.reverse()

            path_x = path_x2 + path_x1
            path_y = path_y2 + path_y1

            return path_x, path_y

        else:
            print('ERROR!')
            return [],[]


    def random_sampling(self):
        rand_x = random.uniform(self.min_x, self.max_x)
        rand_y = random.uniform(self.min_y, self.max_y)
        return node(rand_x, rand_y)


    def distance(self, point1_x, point1_y, point2_x, point2_y):
        return math.hypot(point1_x - point2_x, point1_y - point2_y)


    def find_nearest_node(self, node_list, node):
        dist =[]
        start1 = time.time()
        for nd in node_list:
            dist.append(self.distance(nd.x, nd.y, node.x, node.y))
        end1 = time.time()
        #print("1  ", end1-start1)

        start2 = time.time()
        min_index = dist.index(min(dist))
        end2 = time.time()
        #print("2  ", end2-start2)
        return min_index

    def check_ob(self, new_node, nearest_node):
        dist = []
        # for xx, yy in self.obstacle_x, self.obstacle_y:
        #     dist.append(self.distance(nd.x, nd.y, xx, yy))
        for i in range(len(self.obstacle_x)):
            dist.append(self.distance(new_node.x, new_node.y, self.obstacle_x[i], self.obstacle_y[i]))
        for d in dist:
            if d < self.robot_size + self.avoid_distance:
                return True

        return False








