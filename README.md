# dataMPC_parking
Data driven MPC in parking lot

Xu Shen, xu_shen@berkeley.edu

## Dependencies
### MATLAB:
1. MPT Toolbox (For solving and viz)
2. Parallel Computing Toolbox (Only for generating dataset)
3. Deep Learning Toolbox (For network training and predicting)

## Change log
### 08/04/2020
1. Added NN classifier for strategy prediction (`nn_clas.m`) and added one-hot label variables for it (`load_train_classify.m`)
2. Modified predict and viz script to show strategy posterior probability at the same time (`predict_vs_opt_strategy.m`)

### 08/03/2020
1. Added the strategy generation (`nominal_M PC/check_strategy_labels.m` and `nominal_MPC/generat_strategy_data.m`) and its parallel datagen version for constructing dataset (`learning/strategy_datagen.m`).
2. Changed `learning/loda_train_model.m` to `learning/load_train_regression.m` for the specific use of regression of hyperplane.
3. Added `load_train_classify.m` for constructing dataset for strategy classification. 
4. Added classifiers: Gaussian SVM (`learning/gSVM.m`), bagged tree (`learning/bagTree.m`), and KNN (`learning/knn.m`)

### 07/31/2020
1. Small modification of saving and loading.
2. Added the movie recording function. (`learning/predict_vs_opt_midpoint.m`)

### 07/30/2020
1. Manually divided the \~80% training set and 20% validation set for NN and GP (`learning/load_train_model.m`)
2. Simplified the GP code, so that it only fits once, output compact model, and offer validation MSE. (`gp.m`)

### 07/29/2020
1. Added the GP training. For every element in the flattened label vector, train one GP for it, with all feature dimensions as the predictor. (`learning/gp.m`)
2. Renamed `learning/train_network.m` into `learning/load_train_model.m` so that user has the option to train either NN or GP. The NN training is also organized into `learning/nn.m`
3. Changed the prediction and viz script so that it can switch between GP and NN. (`learning/predict_vs_opt_midpoint.m`)

### 07/28/2020
1. Changed the `learning/gen_feature_label.m` so that it will switch between mid-point mode or free hyperplane mode
2. Added mid-point into `learning/train_network.m` and seperated the normalization process. (The normalized variable is not used for training now) Also added a new variable `reg_data` for regression toolbox.
3. Created a new script for prediction and visualization in mid-point mode. (`learning/predict_vs_opt_midepoint.m`)

### 07/27/2020
1. Customized the training code (`learning/train_network.m`)
2. Check the network prediction vs opt (`learning/predict_vs_opt.m`)
3. Reorganized the data folder:
	1. `./data/` is the folder to place trajectory examples
	2. `./hyperplane_dataset/` is the folder to place generated hyperplanes for training
	3. `./learning/models/` is the folder to place learned models
4. Added the hyperplane generation v2, where the hyperplanes are forced to go across the middle point of two sets, and slope is the free decision variable. (`nominal_MPC/generate_hyperplane_v2.m` and `learning/hyperplane_datagen_v2.m`)
5. ~~**TODO:** The slope will have the wrapping issue and jumps from pi/2 to -pi/2. Need to smooth it in training feature.~~

### 07/26/2020
1. Reorganized the folder tree, placed all learning related into `./learning/`
2. Generate hyperplanes from all scenarios and make the dataset (`learning/hyperplane_datagen.m`)
3. Train a shallow 2-layer FC network and save the model (`learning/train_network.m`)

### 07/25/2020
1. Generate optimal hyperplanes by solving a SVM problem: The current vehicle vertices are hard constraints, the future vertices are soft with slack var. (`nominal_MPC/generate_hyperplanes.m`, Author: Edward)
2. Generate hyperplane dataset by traversing all data files. (`learning/hyperplane_datagen.m`)

### 07/23/2020
1. Cleaned the code:
	1. Script for Time Invariant Obstacles: `nominal_MPC/path_planning_ti.m`
	2. Script for Time Varying Obstacles: `nominal_MPC/path_planning_tv_CFTOC.m` and `nominal_MPC/*_datagen*`
	3. Scipt for Data loading and visualization: `nominal_MPC/data_viz.m`
	4. Controller Functions: `nominal_MPC/OBCA.m`, `nominal_MPC/OBCA_tv`, `emergency_break.m`, `nominal_MPC/speed_controller.m`
	5. Warm Start Functions: `nominal_MPC/DealWultWS.m`, `nominal_MPC/DualMultWS_tv.m`, `nominal_MPC/hybrid_A_star.m`, `nominal_MPC/unicycleWS.m`
	6. Dynamics and other utils: `nominal_MPC/bikeFE.m`, `nominal_MPC/check_collision_*`, `nominal_MPC/plotCar.m`

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
