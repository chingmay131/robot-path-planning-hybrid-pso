# Path Planning for Mobile Robot Navigation in Unknown Indoor Environments Using Hybrid Metaheuristic Algorithms
1. launch folder
   - **proposed_easy.launch**
     - for path planning and navigation in **Easy** level world using **PSOFS** algorithm
   - **proposed_medium.launch**
     - for path planning and navigation in **Medium** level world using **PSOFS** algorithm
   - **proposed_hard.launch**
     - for path planning and navigation in **Hard** level world using **PSOFS** algorithm
   - **existing1_easy.launch**
     - for path planning and navigation in **Easy** level world using **PSOA** algorithm
   - **existing1_medium.launch**
     - for path planning and navigation in **Medium** level world using **PSOA** algorithm
   - **existing1_hard.launch**
     - for path planning and navigation in **Hard** level world using **PSOA** algorithm
   - **existing2_easy.launch**
     - for path planning and navigation in **Easy** level world using **PSOD** algorithm
   - **existing2_medium.launch**
     - for path planning and navigation in **Medium** level world using **PSOD** algorithm
   - **existing2_hard.launch**
     - for path planning and navigation in **Hard** level world using **PSOD** algorithm

2. scripts folder
   - **PSOFS.py**
     - containing code for the hybrid algorithm of [PSO](https://en.wikipedia.org/wiki/Particle_swarm_optimization) and [Fringe Search](https://en.wikipedia.org/wiki/Fringe_search) algorithms
   - **PSOA.py**
     - containing code for the hybrid algorithm of PSO and [A* Search](https://en.wikipedia.org/wiki/A*_search_algorithm) algorithms
   - **PSOD.py**
     - containing code for the hybrid algorithm of PSO and [D*](https://en.wikipedia.org/wiki/D*) algorithms
   - **ObjectiveFunction.py**
     - containing code for evaluating the length, smoothness, and safety of a path, followed by creating a [pareto optimal](https://en.wikipedia.org/wiki/Pareto_efficiency) solution for the respective path using the returned values 
   - **Obstacle.py**
     - containing code for extracting the center point of each obstacle (wood coloured box) in a world from the respective world file, followed by grouping the obstacles to obtain the corner points for computing path safety
   - **Visualization.py**
     - containing code for visualizing all pareto optimal solutions and paths created using any of these algorithms (PSOFS, PSOA, and PSOD) in any of the worlds (Easy, Medium, and Hard), and the best paths returned by all algorithms for a particular world, e.g., Easy level world
