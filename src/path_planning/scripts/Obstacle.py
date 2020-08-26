# -*- coding: utf-8 -*-
import xmltodict
import os
import ast
import itertools

full_path = os.path.realpath(__file__)
path = os.path.dirname(full_path)

# =============================================================================
# Get list of obstacles from .world file
# =============================================================================
def get_obstacle_list(filename):
    obstacles = []
    
    with open(filename) as fd:
        doc = xmltodict.parse(fd.read())
        
    for artikel in doc['sdf']['world']['model']:
        for tag in artikel['link']['visual']['geometry'].keys():
            if tag == 'box' and \
            artikel['link']['visual']['material']['script']['name'] == 'Gazebo/Wood':
                x = float(artikel['pose']['#text'].split()[0])
                y = float(artikel['pose']['#text'].split()[1])
                obstacles.append((x, y))
        
    txt = filename.split("_")[0]
    f = open("obstacles_"+txt+".txt", "w")
    f.write(str(obstacles))
    f.close()

get_obstacle_list('easy_10x10_wall_world.world')
get_obstacle_list('medium_10x10_wall_world.world')
get_obstacle_list('hard_10x10_wall_world.world')

def transform_obstacles(world_name):
    f = open(path+"/obstacles_"+world_name+".txt", "r")
    obstacles = f.readline()
    f.close()
    obstacles = ast.literal_eval(obstacles)
    processed_obstacles = []
    transformed_obstacles = []
    scale = 0.5
        
    for artikel in obstacles:
        center_x = round(artikel[0] * 2) / 2
        center_y = round(artikel[1] * 2) / 2
        processed_obstacles.append((center_x, center_y))

    def myFunc(e):
      return e[1]
    
    processed_obstacles.sort(key=myFunc)
    
    grouped_obstacles = [list(g) for _, g in itertools.groupby(processed_obstacles, lambda x: x[1])]
        
    for go in grouped_obstacles:
        go2 = [ go[i:i+2] for i in range(0, len(go), 2) ]

        for go3 in go2:
            print(go3)

            first = go3[0]
            firstx = first[1]
            firsty = first[0]
            
            topleftx = firstx - scale # top left
            toplefty = firsty - scale
                
            toprightx = firstx + scale # top right
            toprighty = firsty - scale
            second = go3[1]
            secondx = second[1]
            secondy = second[0]
            
            bottomleftx = secondx - scale # bottom left
            bottomlefty = secondy + scale
                
            bottomrightx = secondx + scale # bottom right
            bottomrighty = secondy + scale

            print((topleftx, toplefty, toprightx, toprighty, \
            bottomleftx, bottomlefty, bottomrightx, bottomrighty))
            transformed_obstacles.append((topleftx, toplefty, toprightx, toprighty, \
            bottomleftx, bottomlefty, bottomrightx, bottomrighty))

    f = open("transformed_obstacles_"+world_name+".txt", "w")
    f.write(str(transformed_obstacles))
    f.close()
    
# transform_obstacles("easy")
# transform_obstacles("medium")
# transform_obstacles("hard")