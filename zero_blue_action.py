from action import Action
from vision import  Vision
import time
import math
class zero_blue_action(object):
    def __init__(self):
        self.vmax = 2000
        self.wmax = 1
        self.eps_angle = 0.08
        self.eps_position = 5


    def simplify_path(self,path_x,path_y):
        i = 0

        while i < len(path_x) - 2:
            if path_x[i+1] != path_x[i] and path_x[i+2] != path_x[i+1]:
                degree1 = (path_y[i+1] - path_y[i]) / (path_x[i+1] - path_x[i])
                degree2 = (path_y[i+2] - path_y[i+1]) / (path_x[i+2] - path_x[i+1])
                if degree1 == degree2:
                    del path_x[i + 1]
                    del path_y[i + 1]
                else:
                    i = i + 1
            elif path_x[i+1] == path_x[i] and path_x[i+2] == path_x[i+1]:
                del path_x[i+1]
                del path_y[i+1]
            else:
                i = i + 1

        # i = 0
        # while i < len(path_x) - 1:
        #     if math.sqrt((path_x[i + 1] - path_x[i]) ** 2 + (path_y[i + 1] - path_y[i]) ** 2) <= 400:
        #         del path_x[i + 1]
        #         del path_y[i + 1]
        #     else:
        #         i = i + 1
        return path_x,path_y


    def simplify_path2(self, path_x, path_y):
        i = 0
        while i < len(path_x) - 2:
            if abs(path_x[i+2]-path_x[i]) + abs(path_y[i+2]-path_y[i]) <= 10 * 200:
                del path_x[i+1]
                del path_y[i+1]
            else:
                i = i + 1
        i = 0
        while i < len(path_x) - 1:
            if math.sqrt((path_x[i+1] - path_x[i]) ** 2 + (path_y[i+1] - path_y[i]) ** 2) <= 600:
                del path_x[i + 1]
                del path_y[i + 1]
            else:
                i = i + 1
        return path_x, path_y



    def move_to_goal(self, action, vision, path_x, path_y):
        for i in range(len(path_x) - 1):
            self.move_to_next_joint(action, vision,path_x[i + 1], path_y[i + 1])
            print(i + 1)
            #print(math.hypot(vision.my_robot.x - path_x[i+1], vision.my_robot.y - path_y[i+1]))
    def move_to_next_joint(self, action,vision, path_x, path_y):
        # use iteration to make sure the movement to be smooth
        #print('in')
        while math.sqrt((abs(vision.my_robot.x - path_x)) ** 2 + (abs(vision.my_robot.y - path_y)) ** 2) > 1200:
            #print('in heng')
            self.detect_ob_and_escape(action, vision)
            self.detect_ob_and_escape(action,vision)
            self.move_to_next_joint(action,vision, (vision.my_robot.x + path_x) * 0.5, (vision.my_robot.y + path_y) * 0.5)
        #self.rotate_smoothly_to_certain_angle(action,vision,math.atan2(path_y - vision.my_robot.y, path_x - vision.my_robot.x)/180*math.pi)

        #print('in2')
        angle = math.atan2(path_y - vision.my_robot.y, path_x - vision.my_robot.x)
        delta_angle = angle - vision.my_robot.orientation

        #print('in3')
        flag = 0
        self.detect_ob_and_escape(action, vision)
        if abs(angle - vision.my_robot.orientation) > 0.05:
            self.detect_ob_and_escape(action, vision)
            #print('in4')
            if 0 < delta_angle < math.pi or delta_angle < -math.pi:
                for i in range(1, 61):
                    action.sendCommand(0, 0, 0.015 * i)
                    flag = 1
            else:
                for i in range(1, 61):
                    action.sendCommand(0, 0, -1*0.015 * i)


            delta_angle = angle - vision.my_robot.orientation
            if 0 < delta_angle < math.pi or delta_angle < -math.pi:
                action.sendCommand(vx=0, vy=0, vw=0.9)
            else:
                action.sendCommand(vx=0,vy=0,vw=-0.9)
            threshold_angle = 0.05 + 0.1 * abs(delta_angle)
            #print('angle1')
            while abs(angle - vision.my_robot.orientation) > 0.05:
                time.sleep(0.001)
                #print(abs(math.atan2(path_y - vision.my_robot.y, path_x - vision.my_robot.x) - vision.my_robot.orientation))

                angle = math.atan2(path_y - vision.my_robot.y, path_x - vision.my_robot.x)
                # if angle < 0:
                #     angle = -angle + 2*math.pi
            if flag == 1:
                for i in range(61):
                    action.sendCommand(vx=0, vy=0, vw=(60 - i) * 0.015)
            else:
                for i in range(61):
                    action.sendCommand(vx=0, vy=0, vw=-1*(60 - i) * 0.015)
            self.detect_ob_and_escape(action, vision)
            #print('angle')
            # time.sleep(1/60)
        action.sendCommand(0, 0, 0)
        time.sleep(0.2)

        if math.hypot(vision.my_robot.x - path_x , vision.my_robot.y - path_y) > 100:
            #self.move_smoothly_to_certain_position(action,vision, path_x, path_y)
            #distance = math.sqrt((vision.my_robot.x - path_x) ** 2 + (vision.my_robot.y - path_y) ** 2)
            # acceleration
            #print('path1')
            for i in range(1, 101):
                action.sendCommand(vx=15 * i, vy=0, vw=0)
            # constant velocity
            #print('path2')
            #action.sendCommand(vx=self.vmax, vy=0, vw=0)
            action.sendCommand(1500,0,0)
            #print(path_x,path_y)
            while math.hypot(vision.my_robot.x - path_x, vision.my_robot.y - path_y) > 200:
                if self.detect_ob_and_escape(action, vision) == True:
                    for i in range(1, 101):
                        action.sendCommand(vx=15 * i, vy=0, vw=0)
                    action.sendCommand(1500,0,0)
                time.sleep(0.001)
            # deceleration
            #print('path3')
            for i in range(1, 101):
                action.sendCommand(vx=1500-i*15, vy=0, vw=0)
                time.sleep(0.001)
            action.sendCommand(0, 0, 0)
            time.sleep(0.2)
            self.detect_ob_and_escape(action, vision)
            #print('path')
        #time.sleep(0.2)


    def detect_ob_and_escape(self,action,vision):
        obstacle_x = []
        obstacle_y = []
        # for robot_blue in vision.blue_robot:
        #     if robot_blue.id > 0:
        #         obstacle_x.append(robot_blue.x)
        #         obstacle_y.append(robot_blue.y)
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.id == 3 or robot_yellow.id == 4 or robot_yellow.id == 6 or robot_yellow.id == 9 or robot_yellow.id == 12:
                obstacle_x.append(robot_yellow.x)
                obstacle_y.append(robot_yellow.y)

        for i in range(len(obstacle_x)):
            angle = math.atan2(obstacle_y[i] - vision.my_robot.y, obstacle_x[i] - vision.my_robot.x)
            if math.hypot(vision.my_robot.x - obstacle_x[3], vision.my_robot.y - obstacle_y[3]) < 520:
                print('Collision')
                velocity_x = vision.my_robot.vel_x
                step = round(abs(velocity_x) / 15)
                for i in range(1, step + 1):
                    action.sendCommand(velocity_x - i * 15)
                action.sendCommand(0, 0, 0)
                time.sleep(1)
                del obstacle_x[3]
                del obstacle_y[3]
                # print(math.hypot(obstacle_x[i] - vision.my_robot.x,obstacle_y[i] - vision.my_robot.y))
                return True
            if 200 < math.hypot(vision.my_robot.x - obstacle_x[i], vision.my_robot.y - obstacle_y[i]) < 350:
                if ((angle <= -math.pi/2 or angle >= math.pi/2) and obstacle_x[i] - vision.my_robot.x < 0) or ((0 < angle < math.pi/2 or 0 > angle > -math.pi/2) and obstacle_x[i] - vision.my_robot.x > 0):
                    print('Collision')
                    velocity_x = vision.my_robot.vel_x
                    step = round(abs(velocity_x) / 15)
                    for i in range(1,step+1):
                        action.sendCommand(velocity_x-i*15)
                    action.sendCommand(0,0,0)
                    time.sleep(1)
                    #print(math.hypot(obstacle_x[i] - vision.my_robot.x,obstacle_y[i] - vision.my_robot.y))
                    return True
                if math.hypot(vision.my_robot.x - obstacle_x[i], vision.my_robot.y - obstacle_y[i]) <= 200:
                    print('Collision2')
                    velocity_x = vision.my_robot.vel_x
                    step = round(abs(velocity_x) / 15)
                    for i in range(1, step + 1 + 30):
                        action.sendCommand(velocity_x - i * 15)
                    action.sendCommand(-450,0,0)
                    time.sleep(1)
                    for i in range(1,31):
                        action.sendCommand(-450+i*15,0,0)
                    action.sendCommand(0,0,0)
                    angle = math.atan2(vision.my_robot.y - obstacle_y[i],vision.my_robot.x - obstacle_x[i])
                    print(math.hypot(obstacle_x[i] - vision.my_robot.x,obstacle_y[i] - vision.my_robot.y))

                    return True

        return False




