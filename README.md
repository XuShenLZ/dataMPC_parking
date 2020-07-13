# dataMPC_parking
Data driven MPC in parking lot

Xu Shen, xu_shen@berkeley.edu

## Change log
### 07/12/2020
1. HOBCA with time variant obstacle formulation. (But in the test, the obstacle remains static for now.)
2. Use the simplified vehicle model and collision avoidance constraints for WS, rather than Hybrid A\*. Because the Hybrid A\* may produce reverse motion due to the collision buffer.
3. Changed some function API.

### 07/06/2020
1. HOBCA with multiple obstacles added.
2. TODO: dynamic object.

### 07/02/2020
1. HOBCA deployed in MATLAB where only one static obstacle is present. The file is `path_planning.m`.
2. Workflow:
	1. Obstacle is firstly defined as Polyhedron;
	2. The figure is captured as a frame for Hybrid A* toolbox;
	3. The planned path is smoothened;
	4. Dual multipliers are firstly warm started;
	5. The warm starting states are also computed;
	6. The main optimization is solved
3. TODO: Add num of obstacles; Think about dynamic object.
