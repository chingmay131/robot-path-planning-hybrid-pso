#! /usr/bin/env python
# -*- coding: utf-8 -*-
from PSO import PSO
from DStar import DStar
from ObjectiveFunction import ObjectiveFunction
import os
import math
import random
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
    rospy.init_node('psodstar')
    
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
        PSODStar(initial_position_, obstacles, desired_position_)
            
        rate.sleep()

if __name__ == "__main__":
    main()

class PSODStar:
  def __init__(self, robot_initial_position, obstacles, destination):
    self.robot_initial_position = robot_initial_position
    self.obstacles = obstacles
    self.destination = destination
    self.algorithm()

  def algorithm(self):
    path_points = []
    path_points.append(self.robot_initial_position)
    robot_current_position = self.robot_initial_position

    # define the number of via points and parameters for PSO algorithm
    via_points = 3
    v = 0
    w = 0.5
    c1 = 1
    c2 = 2

    # pathfinding using D* algorithm
    while robot_current_position != self.destination:
      turning_point = []
      turning_point = DStar(robot_current_position)
      path_points.append(turning_point)
      robot_current_position = turning_point

    # add robot_destination to path_points
    path_points.append(self.destination)

    for i in range(0, via_points):
      # select a via point from the obtained path randomly
      random_turning_point = random.choice(path_points)

      # identify random_turning_point index

      # update local and global values using PSO algorithm
      PSO(random_turning_point)

      # interpolate the via point by spline equation
      interpolated_turning_point = solve_spline_equation(random_turning_point)

      # replace the turning point with the interpolated turning point at the same index

    # create solution using path_points and world_obstacles
    solution = ObjectiveFunction().create_pareto_optimal_solution(self.obstacles, path_points)
    
    # return solution, along with path_points
    solution_path = [solution, path_points]

    return solution_path