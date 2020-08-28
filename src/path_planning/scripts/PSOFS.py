#! /usr/bin/env python
# -*- coding: utf-8 -*-
from PSO import PSO
from FringeSearch import FringeSearch
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
    rospy.init_node('psofs')
    
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
        PSOFS(initial_position_, obstacles, desired_position_)
            
        rate.sleep()

if __name__ == "__main__":
    main()

class PSOFS:
  # initialize robot_starting_point, world_obstacles, robot_destination
  def __init__(self, robot_initial_position, obstacles, destination):
    self.robot_initial_position = robot_initial_position
    self.obstacles = obstacles
    self.destination = destination
    self.algorithm()

  def algorithm(self):
    # create an empty list named path_points
    path_points = []

    # add robot_starting_point to path_points
    path_points.append(self.robot_initial_position)

    robot_current_position = self.robot_initial_position
    limit = 1

    while robot_current_position != self.destination:           
      turning_point = []

      # pathfinding using PSO algorithm
      turning_point = PSO(robot_curent_position)

      # if the distance between the robot's current position and any obstacle is less than 1 meter
      if detect_obstacle() < limit:
        # set robot_current_position to the position where it almost collides with obstacles
        robot_current_position = current_position_

        # pathfinding using Fringe Search algorithm
        turning_point = FringeSearch(robot_curent_position)

      # if the robot reaches the turning_point
      if current_position_ == turning_point:
        # add turning_point to path_point
        path_points.append(turning_point)

        robot_current_position = turning_point

    # add robot_destination to path_points
    path_points.append(self.destination)

    # create solution using path_points and world_obstacles
    solution = ObjectiveFunction().create_pareto_optimal_solution(self.obstacles, path_points)
    
    # return solution, along with path_points
    solution_path = [solution, path_points]

    return solution_path