#! /usr/bin/env python
# -*- coding: utf-8 -*-
from PSO import PSO
from AStar import AStar
from ObjectiveFunction import ObjectiveFunction
import os
import math
# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *

initial_position_ = Point()
initial_position_.x = rospy.get_param('start_x')
initial_position_.y = rospy.get_param('start_y')
initial_position_.z = 0
current_position_ = Point()
current_position_.x = rospy.get_param('current_x')
current_position_.y = rospy.get_param('current_y')
current_position_.z = 0
desired_position_ = Point()
desired_position_.x = rospy.get_param('goal_x')
desired_position_.y = rospy.get_param('goal_y')
desired_position_.z = 0
world = rospy.get_param('world')
path = os.path.dirname(__file__)
f = open(path+"/"+world+".txt", "r")
obstacles = f.readline()
f.close()

def main():
    rospy.init_node('psoastar')
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rospy.wait_for_service('/gazebo/set_model_state')
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # set robot position
    model_state = ModelState()
    model_state.model_name = 'tb3burger'
    model_state.pose.position.x = initial_position_.x
    model_state.pose.position.y = initial_position_.y
    resp = srv_client_set_model_state(model_state)
    
    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        PSOAStar(initial_position_, obstacles, desired_position_)
            
        rate.sleep()

if __name__ == "__main__":
    main()

class PSOAStar:
  def __init__(self, robot_initial_position, obstacles, destination):
    self.robot_initial_position = robot_initial_position
    self.obstacles = obstacles
    self.destination = destination
    self.algorithm()

  def algorithm(self):
    path_points = []
    path_points.append(self.robot_initial_position)
    robot_current_position = self.robot_initial_position
    v = 0
    w = 0.5
    c1 = 1
    c2 = 2
    r1 = 0
    r2 = 0
    pbest = 0
    gbest = 0
    x = 0

    while robot_current_position != self.destination:
      turning_point = []
      turning_point = PSO(robot_current_position, v)

      # calculate diversity
      diversity = calculate_diversity()

      dlow = 0 # lower threshold of diversity
      dhigh = 0 # upper threshold of diversity
      limit = 1 # distance between the robot's current position and any obstacle

      # if the distance between the robot's current position and any obstacle is less than 1 meter
      if detect_obstacle() < limit:
        # continue planning the path for the robot using A* search algorithm
        robot_current_position = current_position_
        turning_point = AStar(robot_current_position)
      else if diversity > dhigh:
        # the robot will move towards the global best position (attraction)
        v = w * v + c1 * r1 * (pbest - x) + c2 * r2 * (gbest - x)
        robot_current_position = current_position_
        turning_point = PSO(robot_current_position, v)
      else if dlow < diversity < high:
        # the robot will move towards its own best position (combination of attraction and repulsion)
        v = w * v + c1 * r1 * (pbest - x) - c2 * r2 * (gbest - x)
        robot_current_position = current_position_
        turning_point = PSO(robot_current_position, v)
      else if diversity < dlow:
        # the robot will move towards its own best position (repulsion)
        v = -1 * w * v - c1 * r1 * (pbest - x) - c2 * r2 * (gbest - x)
        robot_current_position = current_position_
        turning_point = PSO(robot_current_position, v)

      path_points.append(turning_point)

      # update new position accordingly
      robot_current_position = turning_point

    # add robot_destination to path_points
    path_points.append(self.destination)

    # create solution using path_points and world_obstacles
    solution = ObjectiveFunction().create_pareto_optimal_solution(self.obstacles, path_points)

    # return solution, along with path_points
    solution_path = [solution, path_points]

    return solution_path