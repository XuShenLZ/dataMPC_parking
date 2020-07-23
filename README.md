# dataMPC_parking
Data driven MPC in parking lot

Xu Shen, xu_shen@berkeley.edu

## Dependencies
### MATLAB:
1. MPT Toolbox (For solving and viz)
2. Parallel Computing Toolbox (Only for generating dataset)

## Change log
### 07/23/2020
1. Cleaned the code:
	1. Script for Time Invariant Obstacles: `path_planning_ti.m`
	2. Script for Time Varying Obstacles: `path_planning_tv_CFTOC.m` and `*_datagen*`
	3. Scipt for Data loading and visualization: `data_viz.m`
	4. Controller Functions: `OBCA.m`, `OBCA_tv`, `emergency_break.m`, `speed_controller.m`
	5. Warm Start Functions: `DealWultWS.m`, `DualMultWS_tv.m`, `hybrid_A_star.m`, `unicycleWS.m`
	6. Dynamics and other utils: `bikeFE.m`, `check_collision_*`, `plotCar.m`

### 07/22/2020
1. Added data loading and viz script (`data_viz.m`)

### 07/20/2020
1. Added the parallel computing version for faster dataset generation (`path_planning_datagen_par.m`)

### 07/13/2020
1. Path planning with the CFTOC formulation (`path_planning_tv_CFTOC.m`):
	1. Use the simple reference traj to check the collision. If no collsion, adjust the initial state to produce it.
	2. Try to solve the exact collision avoidance problem. Firstly, use the unicycle model as state WS, and then dual WS, and finally OBCA.
	3. If the exact problem cannot be solved, try to solve a speed profile on ref path for collision avoidance.
	4. If the speed profile cannot be solved too, solve a most conservative emergency break.
2. Iteratively run over all `exp_num`s and generate data with log file (`path_planning_datagen.m`)
3. Minor fix about constraints of the controller.

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
