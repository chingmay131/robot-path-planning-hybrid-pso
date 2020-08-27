# Path Planning for Mobile Robot Navigation in Unknown Indoor Environments Using Hybrid Metaheuristic Algorithms
1. Overview
2. Software Installation
3. How to Use src Folder
4. How to Use bash_scripts Folder

## Overview
A virtual [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) Burger is launched to navigate in 3 different simulated environments named Easy, Medium, and Hard level worlds while planning its path to the destination online using 3 different hybrid metaheuristic algorithms named PSOFS, PSOA, and PSOD.

Easy Level World (containing 4 static obstacles) | Medium Level World (containing 6 static obstacles) | Hard Level World (containing 7 static obstacles)
------------ | ------------- | -------------
<img src="https://github.com/chingmay131/robot-path-planning-hybrid-pso/blob/master/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/easy.jpg"> | <img src="https://github.com/chingmay131/robot-path-planning-hybrid-pso/blob/master/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/medium.jpg"> | <img src="https://github.com/chingmay131/robot-path-planning-hybrid-pso/blob/master/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/hard.jpg">

PSOFS is a hybrid algorithm of [PSO](https://en.wikipedia.org/wiki/Particle_swarm_optimization) algorithm and [Fringe Search](https://en.wikipedia.org/wiki/Fringe_search) algorithm. PSOA is a hybrid algorithm of PSO algorithm and [A* Search](https://en.wikipedia.org/wiki/A*_search_algorithm) algorithm. PSOD is a hybrid algorithm of PSO algorithm and [D*](https://en.wikipedia.org/wiki/D*) algorithm.

## Software Installation
1. Ubuntu 16.04 LTS
   - [Installation guide](https://ubuntu.com/tutorials/install-ubuntu-desktop-1604#1-overview)
2. ROS Kinetic Kame
   - [Installation guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)
   - [Tutorial](http://wiki.ros.org/ROS/Tutorials)
3. Gazebo 7.x
   - [Installation guide](http://gazebosim.org/tutorials?tut=install_ubuntu)
   - [Tutorial](http://gazebosim.org/tutorials)

## How to Use src Folder
1. [Create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) named catkin_ws
2. [Create a ROS package](http://wiki.ros.org/catkin/Tutorials/CreatingPackage) named path_planning
3. Copy [turtlebot3](https://github.com/chingmay131/robot-path-planning-hybrid-pso/tree/master/src/turtlebot3), [turtlebot3_msg](https://github.com/chingmay131/robot-path-planning-hybrid-pso/tree/master/src/turtlebot3_msgs), and [turtlebot3_simulations](https://github.com/chingmay131/robot-path-planning-hybrid-pso/tree/master/src/turtlebot3_simulations) folders to src folder in your newly created catkin workspace - these folders contain the packages for TurtleBot3
4. Copy [launch](https://github.com/chingmay131/robot-path-planning-hybrid-pso/tree/master/src/path_planning/launch) and [scripts](https://github.com/chingmay131/robot-path-planning-hybrid-pso/tree/master/src/path_planning/scripts) folders to your newly created ROS package
5. [Build all packages in your workspace](http://wiki.ros.org/catkin/Tutorials/using_a_workspace)

## How to Use bash_scripts Folder
1. Open Terminal in Ubuntu
2. Change access permissions for all files in bash_scripts folder
```
chmod a+x "bash_scripts/"*
```
3. Change working directory to any of the folders, for example:
```
cd bash_scripts/easy
```
By changing your current working directory to Easy folder, it means that the robot will navigate in Easy level world.

4. Launch Easy level world in Gazebo
```
./easy_world
```
5. Open the second Terminal window and change to the same working directory
6. Run any of the bash scripts in your current working directory, for example:
```
./proposed
```
This bash script will spawn the robot to Easy level world, and the robot will start navigating to its destination in the world by planning the path online using PSOFS algorithm.

**Optional Steps:**

7. Open the third Terminal window and change to the same working directory
8. Launch [rviz](http://wiki.ros.org/rviz) to visualize the robot's laser scan
```
./easy_rviz
```
9. If you would like to restart the navigation in the same world, stop the current process in the second Terminal window.
10. Run any of the bash scripts where the filename ends with '\_delete', for example:
```
./proposed_delete
```
