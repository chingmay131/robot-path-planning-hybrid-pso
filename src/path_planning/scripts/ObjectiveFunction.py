# -*- coding: utf-8 -*-
import math
from scipy import spatial
    
# =============================================================================
# Objective functions
# =============================================================================
class ObjectiveFunction:
    # path length = total distance travelled by the robot
    def compute_path_length(self, path_points):
        total_distance = 0.0
        point_to_point_distances = []
        
        for current_point, next_point in zip(path_points, path_points[1:]):
            distance = HelperFunction().calculate_euclidean_distance(current_point, next_point)
            point_to_point_distances.append(distance)
            
        for dist in point_to_point_distances:
            total_distance += dist
        
        return total_distance
        
    # path smoothness = average angle between two path segments
    def compute_path_smoothness(self, path_points):
        angles = []
        
        for current_point, next_point1, next_point2 in zip(path_points, \
                                                           path_points[1:], path_points[2:]):
            angle_radians = HelperFunction().calculate_angle(current_point, next_point1, next_point2)
            angles.append(angle_radians)
            
        x = sum(math.cos(a) for a in angles)
        y = sum(math.sin(a) for a in angles)
    
        if x == 0 and y == 0:
            raise ValueError("The angle average of the inputs is undefined: %r" % angles)
    
        # To get outputs from -pi to +pi, delete everything but math.atan2() here.
        average_angle = math.fmod(math.atan2(y, x) + 2 * math.pi, 2 * math.pi)
        
        return average_angle
        
    # path safety = shortest distance from a path segment to any obstacle
    def compute_path_safety(self, obstacles, path_points):
        line_to_point_distances = []
        
        for current_point, next_point in zip(path_points, path_points[1:]):
            distance = HelperFunction().calculate_line_to_point_distance(obstacles, \
                                                                         current_point, next_point)
            line_to_point_distances.append(distance)
            
        shortest_distance = min(line_to_point_distances)
        
        return shortest_distance
    
    def create_solution(self, obstacles, path_points):
        solution = []
        
        path_length = round(self.compute_path_length(path_points),2)
        path_smoothness = round(self.compute_path_smoothness(path_points),2)
        path_safety = round(self.compute_path_safety(obstacles, path_points),2)
        solution.append(path_length)
        solution.append(path_smoothness)
        solution.append(path_safety)
        
        return solution
        
# =============================================================================
# Helper functions
# =============================================================================
class HelperFunction:
    # Functions for computing path length
    def calculate_euclidean_distance(self, point1, point2):
        x1 = point1[0]
        y1 = point1[1]
        x2 = point2[0]
        y2 = point2[1]
        distance = math.sqrt(((x2-x1)**2 + (y2-y1)**2))
        
        return distance
    
    # Functions for computing path smoothness
    def slope(self, x1, y1, x2, y2): # Line slope given two points:
        if x2-x1==0.0:
            return 0.0
        else:
            return (y2-y1)/(x2-x1)
    
    def calculate_angle(self, point1, point2, point3):    
        lineA = (point1, point2)
        lineB = (point2, point3)        
        slope1 = self.slope(lineA[0][0], lineA[0][1], lineA[1][0], lineA[1][1])
        slope2 = self.slope(lineB[0][0], lineB[0][1], lineB[1][0], lineB[1][1])
        angle_radians = math.atan((slope2-slope1)/(1+(slope2*slope1)))
        
        return angle_radians
    
    # Functions for computing path safety
    def calculate_midpoint(self, point1, point2):
        x1 = point1[0]
        y1 = point1[1]
        x2 = point2[0]
        y2 = point2[1]
        midpoint = ((x1 + x2)/2, (y1 + y2)/2)
        
        return midpoint

    def dot(self, v,w):
        x,y,z = v
        X,Y,Z = w
        return x*X + y*Y + z*Z
    
    def length(self, v):
        x,y,z = v
        return math.sqrt(x*x + y*y + z*z)
    
    def vector(self, b,e):
        x,y,z = b
        X,Y,Z = e
        return (X-x, Y-y, Z-z)
    
    def unit(self, v):
        x,y,z = v
        mag = self.length(v)
        if x==0.0 and mag==0.0:
            xmag = 0.0
        else:
            xmag = x/mag

        if y==0.0 and mag==0.0:
            ymag = 0.0
        else:
            ymag = y/mag

        if z==0.0 and mag==0.0:
            zmag = 0.0
        else:
            zmag = z/mag

        return (xmag, ymag, zmag)
    
    def distance(self, p0,p1):
        return self.length(self.vector(p0,p1))
    
    def scale(self, v,sc):
        x,y,z = v
        if sc==0.0:
            sc = 0.0
        else:
            sc = 1.0/sc
        return (x * sc, y * sc, z * sc)
    
    def add(self, v,w):
        x,y,z = v
        X,Y,Z = w
        return (x+X, y+Y, z+Z)
    
    def calculate_line_to_point_distance(self, obstacles, start, end):
        # find midpoint of path segment
        midpoint = self.calculate_midpoint(start, end)
        
        # find the nearest obstacle
        nearest_obstacle = obstacles[spatial.KDTree(obstacles).query(midpoint)[1]]
        
        start_x = start[0]
        start_y = start[1]
        start_3d = (start_x, 0.0, start_y)
        end_x = end[0]
        end_y = end[1]
        end_3d = (end_x, 0.0, end_y)
        nearest_obstacle_x = nearest_obstacle[0]
        nearest_obstacle_y = nearest_obstacle[1]
        nearest_obstacle_3d = (nearest_obstacle_x, 0.0, nearest_obstacle_y)
        
        # find distance from path segment to the nearest obstacle
        line_vec = self.vector(start_3d, end_3d)
        pnt_vec = self.vector(start_3d, nearest_obstacle_3d)
        line_len = self.length(line_vec)
        line_unitvec = self.unit(line_vec)
        pnt_vec_scaled = self.scale(pnt_vec, line_len)
        t = self.dot(line_unitvec, pnt_vec_scaled)
        
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
            
        nearest = self.scale(line_vec, t)
        dist = self.distance(nearest, pnt_vec)
        nearest = self.add(nearest, start_3d)
        
        return dist