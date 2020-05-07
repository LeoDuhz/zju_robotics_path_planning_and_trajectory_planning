import numpy
import random
import math
import time
from vision import Vision


class Node(object):
    def __init__(self, x, y, g, h, parent):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = self.g + self.h
        self.parent = parent


class Astar2(object):
    def __init__(self, start_x, start_y, goal_x, goal_y):
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 200
        self.avoid_dist = 400
        self.Grid_Size = 200
        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.openlist = []
        self.closelist = []
        self.obstacle_x = []
        self.obstacle_y = []

    def plan(self, vision):
        # Obstacles
        # obstacle_x = [-999999]

        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                self.obstacle_x.append(robot_blue.x)
                self.obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                self.obstacle_x.append(robot_yellow.x)
                self.obstacle_y.append(robot_yellow.y)

        start = Node(self.start_x, self.start_y, 0, self.calculate_h_value(self.start_x, self.start_y), -1)
        self.openlist.append(start)
        while True:
            e = self.find_min_f_in_openlist()
            #if e.x == self.goal_x and e.y == self.goal_y:
            if math.sqrt((e.x - self.goal_x) ** 2 + (e.y - self.goal_y) ** 2) <= 100:
                self.closelist.append(e)
                break
            self.closelist.append(e)
            del self.openlist[self.find_index_of_e_in_openlist(e)]
            self.search_nearby_grids(e)

        path_x,path_y = self.generate_best_path()
        return path_x, path_y

        # mylist = []
        # for i in range(len(self.closelist)):
        #     mylist.append(self.closelist[i])
        # return mylist

    def calculate_h_value(self, x, y):
        h = abs(x - self.goal_x) + abs(y - self.goal_y)
        return h

    def calculate_g_value(self, x, y):
        g = math.sqrt((x - self.start_x) ** 2 + (y - self.start_y) ** 2)
        return g

    def find_min_f_in_openlist(self):


        # minnode = self.openlist[0]
        # for i in range(len(self.openlist)):
        #     a = self.openlist[i]
        #     if minnode.f > a.f:
        #         minnode = a
        # return minnode

        minnode = self.openlist[0]

        for i in range(len(self.openlist)):
            if not ((self.openlist[i].x > 0 and self.openlist[i].y < 0) or (self.openlist[i].x > 0 and self.openlist[i].y < 2000) or (-2500 < self.openlist[i].x < 0 and self.openlist[i].y < 1600)):
                a = self.openlist[i]
                if minnode.f > a.f:
                    minnode = a
        return minnode

    def in_closelist(self, e):
        for i in range(len(self.closelist)):
            if e.x == self.closelist[i].x and e.y == self.closelist[i].y:
                return True
        return False

    def in_closelist_and_return(self, e):
        for i in range(len(self.closelist)):
            if e.x == self.closelist[i].x and e.y == self.closelist[i].y:
                return self.closelist[i]
        return False

    def in_openlist(self, e):
        for i in range(len(self.openlist)):
            if e.x == self.openlist[i].x and e.y == self.openlist[i].y:
                return True
        return False

    def in_openlist_and_return(self, e):
        for i in range(len(self.openlist)):
            if e.x == self.openlist[i].x and e.y == self.openlist[i].y:
                return self.openlist[i]
        return False

    def in_obstacles(self, a):
        for i in range(len(self.obstacle_x)):
            d = (a.x - self.obstacle_x[i]) ** 2 + (a.y - self.obstacle_y[i]) ** 2
            if d <= self.avoid_dist ** 2:
                return True
        return False

    def in_field(self,a):
        if a.x <= self.maxx and a.x >= self.minx and a.y <= self.maxy and a.y >= self.miny:
            return True
        return False

    def find_index_of_e_in_closelist(self, e):
        for i in range(len(self.closelist)):
            if e.x == self.closelist[i].x and e.y == self.closelist[i].y:
                return i
        #return False  # buzhidaoduibudui

    def find_index_of_e_in_openlist(self, e):
        for i in range(len(self.openlist)):
            if e.x == self.openlist[i].x and e.y == self.openlist[i].y:
                return i


    def search_nearby_grids(self, e):
        xnearby = []
        ynearby = []
        # search upper one
        xnearby.append(e.x)
        ynearby.append(e.y + self.Grid_Size)

        xnearby.append(e.x - self.Grid_Size)
        ynearby.append(e.y + self.Grid_Size)

        xnearby.append(e.x + self.Grid_Size)
        ynearby.append(e.y + self.Grid_Size)

        xnearby.append(e.x + self.Grid_Size)
        ynearby.append(e.y)

        xnearby.append(e.x)
        ynearby.append(e.y - self.Grid_Size)

        xnearby.append(e.x - self.Grid_Size)
        ynearby.append(e.y)

        xnearby.append(e.x + self.Grid_Size)
        ynearby.append(e.y - self.Grid_Size)

        xnearby.append(e.x - self.Grid_Size)
        ynearby.append(e.y - self.Grid_Size)

        for k in range(len(xnearby)):
            if k <= 3:
                tg = e.g + self.Grid_Size
            if k > 3:
                tg = e.g + self.Grid_Size * math.sqrt(2)

            temp = Node(xnearby[k], ynearby[k], 99999999999, self.calculate_h_value(xnearby[k], ynearby[k]), -1)
            if self.in_openlist(temp) or self.in_closelist(temp):
                if self.in_openlist(temp):
                    nearby = self.in_openlist_and_return(temp)
                else:
                    nearby = self.in_closelist_and_return(temp)
            else:
                nearby = Node(xnearby[k], ynearby[k], 99999999999, self.calculate_h_value(xnearby[k], ynearby[k]), -1)
            if self.in_field(nearby):
                if not self.in_obstacles(nearby):
                    if not (self.in_closelist(nearby) and tg >= nearby.g):
                        if (not self.in_openlist(nearby)) or tg < nearby.g:
                            nearby.g = tg
                            nearby.f = nearby.g + nearby.h  # very important
                            nearby.parent = self.find_index_of_e_in_closelist(e)
                            if not self.in_openlist(nearby):
                                self.openlist.append(nearby)



    def generate_best_path(self):
        if len(self.closelist) == 0:
            print('Path not found!')
            return

        path_x = [self.closelist[-1].x]
        path_y = [self.closelist[-1].y]
        i = -1
        while self.closelist[i].parent != -1:
            i = self.closelist[i].parent
            path_x.append(self.closelist[i].x)
            path_y.append(self.closelist[i].y)

        print('Path found!')
        return path_x,path_y

    def reverse_path(self,path_x,path_y):
        reverse_path_x = []
        reverse_path_y = []
        j = -1

        for i in range(len(path_x)):
            reverse_path_x[i] = path_x[j]
            reverse_path_y[i] = path_y[j]
            j = j - 1

        return reverse_path_x,reverse_path_y






    # def generate_best_path(self):
    #     best_path = [self.closelist[-1]]
    #     i = -1
    #     while self.closelist[i].parent != -1:
    #         i = self.closelist[i].parent
    #         best_path.append(self.closelist[i])
    #
    #     return best_path