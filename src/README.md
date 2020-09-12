# (Update in progress) Path Planning for Mobile Robot Navigation in Unknown Indoor Environments Using Hybrid Metaheuristic Algorithms
1. path_planning folder - containing launch files and python scripts for path planning and visualization
2. turtlebot3 folder - containing TurtleBot3 core packages
3. turtlebot3_msgs folder - containing TurtleBot3 packages for messages
4. turtlebot3_simulations folder - containing TurtleBot3 packages for simulations and custom files in the following folders
   - [turtlebot3_gazebo/launch](https://github.com/chingmay131/robot-path-planning-hybrid-pso/tree/master/src/turtlebot3_simulations/turtlebot3_gazebo/launch)
     - **easy_10x10_wall_world.launch** - for launching Easy level world in Gazebo
     - **medium_10x10_wall_world.launch** - for launching Medium level world in Gazebo
     - **hard_10x10_wall_world.launch** - for launching Hard level world in Gazebo
     - **spawn_10x10.launch** - for spawning a robot of your choice in any of the worlds mentioned above at your desired robot pose
   - [turtlebot3_gazebo/worlds](https://github.com/chingmay131/robot-path-planning-hybrid-pso/tree/master/src/turtlebot3_simulations/turtlebot3_gazebo/worlds)
     - **easy_10x10_wall_world.world** - Gazebo world file created for Easy level world
     - **medium_10x10_wall_world.world** - Gazebo world file created for Medium level world
     - **hard_10x10_wall_world.world** - Gazebo world file created for Hard level world
