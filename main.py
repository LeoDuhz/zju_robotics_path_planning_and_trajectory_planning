from vision import Vision
from action import Action
from debug import Debugger
from astar2 import Astar2
from zero_blue_action import zero_blue_action
import time
import math

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    zero = zero_blue_action()
    #astar = Astar()
    start_x = 2400
    start_y = 1500
    goal_x = -2400
    goal_y = -1500


    while True:
        time.sleep(1)

        start_x, start_y = 2400,1500
        #start_x, start_y = vision.my_robot.x, vision.my_robot.y
        goal_x, goal_y = -2400, -1500
        # goal_x.append = [-2400]
        # goal_y.append = [-1500]
        astar2 = Astar2(start_x, start_y, goal_x, goal_y)

        path_x, path_y = astar2.plan(vision)
        path_x, path_y = zero.simplify_path(path_x=path_x, path_y=path_y)
        path_x.reverse()
        path_y.reverse()
        print(path_x)
        print(path_y)

        #action.controlObs(vision)
        time.sleep(0.2)
        action.sendCommand(vx=0, vy=0, vw=0)
        #debugger.draw_all(path_x=path_x,path_y=path_y,robot_x=vision.my_robot.x,robot_y=vision.my_robot.y)
        path_x,path_y = zero.simplify_path2(path_x=path_x,path_y=path_y)
        debugger.draw_all(path_x=path_x,path_y=path_y,robot_x=vision.my_robot.x,robot_y=vision.my_robot.y)
        #action.move_to_goal(vision,path_x,path_y)
        while True:
            zero.move_to_goal(action,vision,path_x,path_y)
            print('achieve goal:', math.hypot(vision.my_robot.x - path_x[-1], vision.my_robot.y - path_y[-1]))
            path_x.reverse()
            path_y.reverse()

        time.sleep(0.005)

        # time.sleep(1)
        #
        # #start_x, start_y = vision.my_robot.x, vision.my_robot.y
        # goal_x, goal_y = -2400, -1500
        # start_x, start_y = 2400, 1500
        # while True:
        #     astar2 = Astar2(start_x, start_y, goal_x, goal_y)
        #
        #     path_x, path_y = astar2.plan(vision)
        #     path_x, path_y = zero.simplify_path(path_x=path_x, path_y=path_y)
        #     path_x.reverse()
        #     path_y.reverse()
        #
        #     # action.controlObs(vision)
        #     time.sleep(0.2)
        #     action.sendCommand(vx=0, vy=0, vw=0)
        #     # debugger.draw_all(path_x=path_x,path_y=path_y,robot_x=vision.my_robot.x,robot_y=vision.my_robot.y)
        #     #path_x, path_y = zero.simplify_path2(path_x=path_x, path_y=path_y)
        #     debugger.draw_all(path_x=path_x, path_y=path_y, robot_x=vision.my_robot.x, robot_y=vision.my_robot.y)
        #     while True:
        #         action.follow_traj(vision,path_x,path_y)
        #         path_x.reverse()
        #         path_y.reverse()