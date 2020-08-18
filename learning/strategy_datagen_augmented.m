clear('all');
close('all');
clc

%% ====== Generate data

all_nums = 486;
N = 20;

training_set = cell(1, all_nums);

% Traverse all data files
parfor exp_num = 1:all_nums
	training_set{1, exp_num} = par_strategy_gen(exp_num, N);
end

dataset_filename = ['../hyperplane_dataset/strategy_data_', datestr(now,'yyyy-mm-dd_HH-MM'), '_aug.mat'];

save(dataset_filename, 'training_set')


%% par_strategy_gen: function description
% Augment data set by reflecting trajectory about the x axis
function [training_data] = par_strategy_gen(exp_num, N)
	fprintf('Generating strategy for Exp_num #%d\n', exp_num)

	exp_file = strcat('../data/exp_num_', num2str(exp_num), '.mat');
	load(exp_file)

	T = length(TV.t); % Length of data

	if problem_type == "Infeasible"
	    disp('Solution in file was infeasible, skipping data generation')
	    training_data = [];
	    return
	end

	min_x_diff = inf;
	min_x_idx = -1;
	rel_y = 0;
	for i = 1:T-N
	    % Get x, y, heading, and velocity from ego vehicle at current timestep
	    EV_x = EV.traj(1,i);
	    EV_y = -EV.traj(2,i);
	    EV_th = -EV.traj(3,i);
	    EV_v = EV.traj(4,i);
	    
	    EV_curr = [EV_x; EV_y; EV_th; EV_v*cos(EV_th); EV_v*sin(EV_th)];
	    
	    % Get x, y, heading, and velocity from target vehicle over prediction
	    % horizon
	    TV_x = TV.x(i:i+N);
	    TV_y = -TV.y(i:i+N);
	    TV_th = -TV.heading(i:i+N);
	    TV_v = TV.v(i:i+N);
	    
	    TV_pred = [TV_x, TV_y, TV_th, TV_v.*cos(TV_th), TV_v.*sin(TV_th)]';
	    
	    % Get target vehicle trajectory relative to ego vehicle state
	    rel_state = TV_pred - EV_curr;
	    
	    if abs(EV_x - TV_x(1)) < min_x_diff
	        min_x_diff = abs(EV_x - TV_x(1));
	        min_x_idx = i;
	        rel_y = TV_y(1) - EV_y;
	    end
	    
	    training_data(i).X = rel_state;
	end
	    
	if problem_type == 'Emergency Break'
	    label = "Y"; % Yield
	elseif problem_type == 'Successful Maneuver' || problem_type == 'Collision Free'
	    if rel_y > 0
	        label = "R"; % Right
	    else
	        label = "L"; % Left
	    end
	end

	for i = 1:length(training_data)
	    training_data(i).Y = label;
	end
end