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
    def __init__(self, N_sample=3000):
        self.N_sample = N_sample
        self.threshold = 300
        self.min_x = -6000
        self.max_x = 6000
        self.min_y = -4500
        self.max_y = 4500
        self.goal_x = 3500
        self.goal_y = 2500
        self.expand_length = 200
        self.robot_size = 200
        self.avoid_distance = 200
        self.obstacle_x = [-999999]
        self.obstacle_y = [-999999]

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
        node_list = [start_node]

        for i in range(self.N_sample):
            flag = 0
            rand = self.random_sampling()

            min_index = self.find_nearest_node(node_list, rand)
            nearest_node = node_list[min_index]

            theta = math.atan2(rand.y-nearest_node.y, rand.x-nearest_node.x)
            for i in range(len(self.obstacle_x)):
                if self.distance(vision.my_robot.x, vision.my_robot.y, self.obstacle_x[i], self.obstacle_y[i]) < 200:
                    flag = 1
                    break
            if flag == 1 and min_index == 0:
                new_x = nearest_node.x + 700 * math.cos(theta)
                new_y = nearest_node.y + 700 * math.sin(theta)
                new_node = node(new_x, new_y, min_index)
            else:
                new_x = nearest_node.x + self.expand_length * math.cos(theta)
                new_y = nearest_node.y + self.expand_length * math.sin(theta)
                new_node = node(new_x, new_y, min_index)

            if self.check_ob(new_node, nearest_node):
                continue

            node_list.append(new_node)

            if self.distance(new_node.x, new_node.y, goal_x, goal_y) < self.threshold:
                print('Path Found!')
                flag = 1
                break
        path_x = []
        path_y = []
        index = len(node_list) - 1

        start3 = time.time()
        while node_list[index].parent is not None:
            path_x.append(node_list[index].x)
            path_y.append(node_list[index].y)
            index = node_list[index].parent


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
        eps = 200
        dist = []
        # for index in range(len(self.obstacle_x)):

        #     if self.distance(self.obstacle_x[index], self.obstacle_y[index], self.goal_x, self.goal_y) < eps:
        #         del self.obstacle_x[index]
        #         del self.obstacle_y[index]
        #         continue
        end_obstacle = []
        for index in range(len(self.obstacle_x)):
            if self.distance(self.obstacle_x[index], self.obstacle_y[index], self.goal_x, self.goal_y) < eps:
                end_obstacle.append(index)

        for i in range(len(self.obstacle_x)):
            dist.append(self.distance(new_node.x, new_node.y, self.obstacle_x[i], self.obstacle_y[i]))
        
        i = 0
        for d in dist:
            if d < self.robot_size + self.avoid_distance and i  not in end_obstacle :
                return True
            elif i in end_obstacle:
                if d < self.robot_size + 50:
                    return True
            i += 1
        return False








