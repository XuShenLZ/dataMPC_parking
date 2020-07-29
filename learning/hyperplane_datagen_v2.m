%% Generate hyperplane training data from all traj examples
% the core algorithm is copied from `generate_hyperplances_v2.m`
% v2: the hyperplanes always go across the middle point of two sets
clear('all');
close('all')
clc

all_nums = 486;
N = 20;

training_set = cell(1, all_nums);

% Traverse all data files
parfor exp_num = 1:all_nums
	training_set{1, exp_num} = par_hpp_gen(exp_num, N);
end

dataset_filename = ['../hyperplane_dataset/hpp_data_v2_', datestr(now,'yyyy-mm-dd_HH-MM'), '.mat'];

save(dataset_filename, 'training_set')

%% par_hpp_gen: solve the hyperplanes and construct the training data point
function [training_data] = par_hpp_gen(exp_num, N)
	fprintf('\n====\nSolving Exp_num: %d\n', exp_num)

	file_name = ['../data/exp_num_', num2str(exp_num), '.mat'];
	load(file_name);

	% Do not include bad examples
	if problem_type == "Infeasible"
		fprintf('Exp_num: %d, Problem Type Infeasible, Skipped\n', exp_num);
		training_data = [];
		return
	end

	T = length(TV.t); % Length of data

	last_w = zeros(2,1);
	last_b = 0;

	for i = 1:T-N
	% for i = 1:100
	    % fprintf('Solving for hyperplane at time step %i\n', i)
	    
	    % Get x, y, heading from ego and target vehicles
	    EV_x = EV.traj(1,i:i+N)';
	    EV_y = EV.traj(2,i:i+N)';
	    EV_th = EV.traj(3,i:i+N)';
	    
	    TV_x = TV.x(i:i+N);
	    TV_y = TV.y(i:i+N);
	    TV_th = TV.heading(i:i+N);
	    
	    % Get ego vehicle vertices at current time step
	    EV_Vx_curr = [EV_x(1) + EV.length/2*cos(EV_th(1)) - EV.width/2*sin(EV_th(1));
			  EV_x(1) + EV.length/2*cos(EV_th(1)) + EV.width/2*sin(EV_th(1));
			  EV_x(1) - EV.length/2*cos(EV_th(1)) + EV.width/2*sin(EV_th(1));
			  EV_x(1) - EV.length/2*cos(EV_th(1)) - EV.width/2*sin(EV_th(1))];
	    EV_Vy_curr = [EV_y(1) + EV.length/2*sin(EV_th(1)) + EV.width/2*cos(EV_th(1));
			  EV_y(1) + EV.length/2*sin(EV_th(1)) - EV.width/2*cos(EV_th(1));
			  EV_y(1) - EV.length/2*sin(EV_th(1)) - EV.width/2*cos(EV_th(1));
			  EV_y(1) - EV.length/2*sin(EV_th(1)) + EV.width/2*cos(EV_th(1))];
	    EV_V_curr = [EV_Vx_curr, EV_Vy_curr];
	    
	    % Get ego vehicle vertices along prediction horizon
	    EV_Vx_pred = [EV_x(2:end) + EV.length/2*cos(EV_th(2:end)) - EV.width/2*sin(EV_th(2:end));
			  EV_x(2:end) + EV.length/2*cos(EV_th(2:end)) + EV.width/2*sin(EV_th(2:end));
			  EV_x(2:end) - EV.length/2*cos(EV_th(2:end)) + EV.width/2*sin(EV_th(2:end));
			  EV_x(2:end) - EV.length/2*cos(EV_th(2:end)) - EV.width/2*sin(EV_th(2:end))];

		EV_Vy_pred = [EV_y(2:end) + EV.length/2*sin(EV_th(2:end)) + EV.width/2*cos(EV_th(2:end));
			  EV_y(2:end) + EV.length/2*sin(EV_th(2:end)) - EV.width/2*cos(EV_th(2:end));
			  EV_y(2:end) - EV.length/2*sin(EV_th(2:end)) - EV.width/2*cos(EV_th(2:end));
			  EV_y(2:end) - EV.length/2*sin(EV_th(2:end)) + EV.width/2*cos(EV_th(2:end))];
	    EV_V_pred = [EV_Vx_pred, EV_Vy_pred];
	    
	    % Get target vehicle vertices at current time step
	    TV_Vx_curr = [TV_x(1) + TV.length/2*cos(TV_th(1)) - TV.width/2*sin(TV_th(1));
			  TV_x(1) + TV.length/2*cos(TV_th(1)) + TV.width/2*sin(TV_th(1));
			  TV_x(1) - TV.length/2*cos(TV_th(1)) + TV.width/2*sin(TV_th(1));
			  TV_x(1) - TV.length/2*cos(TV_th(1)) - TV.width/2*sin(TV_th(1))];

		TV_Vy_curr = [TV_y(1) + TV.length/2*sin(TV_th(1)) + TV.width/2*cos(TV_th(1));
			  TV_y(1) + TV.length/2*sin(TV_th(1)) - TV.width/2*cos(TV_th(1));
			  TV_y(1) - TV.length/2*sin(TV_th(1)) - TV.width/2*cos(TV_th(1));
			  TV_y(1) - TV.length/2*sin(TV_th(1)) + TV.width/2*cos(TV_th(1))];
	    TV_V_curr = [TV_Vx_curr, TV_Vy_curr];
	    
	    % Get target vehicle vertices along prediction horizon
	    TV_Vx_pred = [TV_x(2:end) + TV.length/2*cos(TV_th(2:end)) - TV.width/2*sin(TV_th(2:end));
			  TV_x(2:end) + TV.length/2*cos(TV_th(2:end)) + TV.width/2*sin(TV_th(2:end));
			  TV_x(2:end) - TV.length/2*cos(TV_th(2:end)) + TV.width/2*sin(TV_th(2:end));
			  TV_x(2:end) - TV.length/2*cos(TV_th(2:end)) - TV.width/2*sin(TV_th(2:end))];

		TV_Vy_pred = [TV_y(2:end) + TV.length/2*sin(TV_th(2:end)) + TV.width/2*cos(TV_th(2:end));
			  TV_y(2:end) + TV.length/2*sin(TV_th(2:end)) - TV.width/2*cos(TV_th(2:end));
			  TV_y(2:end) - TV.length/2*sin(TV_th(2:end)) - TV.width/2*cos(TV_th(2:end));
			  TV_y(2:end) - TV.length/2*sin(TV_th(2:end)) + TV.width/2*cos(TV_th(2:end))];
	    TV_V_pred = [TV_Vx_pred, TV_Vy_pred];
	    
	    % Split into hard constraint and soft constraint sets
	    hard_X = [EV_V_curr; TV_V_curr];
	    hard_Y = [ones(size(EV_V_curr,1),1); -ones(size(TV_V_curr,1),1)];
	    soft_X = [EV_V_pred; TV_V_pred];
	    soft_Y = [ones(size(EV_V_pred,1),1); -ones(size(TV_V_pred,1),1)];
	    
	    % Compute midpoint between current ego and target vehicle occupied
	    % areas
	    EV_P = Polyhedron('V', EV_V_curr);
	    TV_P = Polyhedron('V', TV_V_curr);
	    s = EV_P.distance(TV_P);
	    m = (s.y + s.x)/2;
	    
	%     m = fitcsvm(svm_X, svm_Y);
	%     w = m.Beta;
	%     b = m.Bias;
	%     hyp_y = [-10, 10];
	%     hyp_x = -(w(2)*hyp_y+b)/w(1);
	    
	    % Initialize decision variables
	    w = sdpvar(2,1);
	    b = sdpvar(1);
	    slack = sdpvar(length(soft_Y),1);
	    
	%     objective = norm(w)^2 + w(1)^2/w(2)^2 + 1e6*norm(slack)^2;
	%     constraints = [(hard_X*w+b).*hard_Y >= 0, (soft_X*w+b).*soft_Y >= 0+slack];
	%     options = sdpsettings('solver', 'ipopt');
	%     optimize(constraints, objective, options)
	    
	    % Objective is to minimize the hyperplane norm and the change in the
	    % hyperplane between time steps
	    if i == 1
	%         objective = norm(w)^2 + 1e6*norm(slack)^2;
	        objective = 1e6*norm(slack)^2;
	    else
	%         objective = norm(w)^2 + 1e6*norm(slack)^2 + 100*(norm(w-last_w)^2 + (b-last_b)^2);
	%         objective = norm(w)^2 + 1e6*norm(slack)^2;
	        objective = 1e6*norm(slack)^2 + 100*(norm(w-last_w)^2 + (b-last_b)^2);
	    end
	    
	    % Hard constraints on separating the ego and target vehicle vertices at
	    % the current time step. This should always be feasible given that the
	    % the trajectories are collision free. Soft constraints on separating
	    % the ego and target vehicle vertices along the prediction horizon
	    constraints = [(hard_X*w+b).*hard_Y >= 0, (soft_X*w+b).*soft_Y >= slack];
	    
	    % Constrain the hyperplane to pass through the midpoint
	    constraints = [constraints, m'*w+b == 0];
	    
	    options = sdpsettings('solver', 'quadprog', 'verbose', 0);
	    status = optimize(constraints, objective, options);
	    
	    if status.problem ~= 0
		    fprintf('Exp_num: %d, Error: %s\n', exp_num, yalmiperror(status.problem))
	        training_data = [];
	        return
	    end
	    w = value(w);
	    b = value(b);
	    
	    % Save in struct array
	    training_data(i).hyperplane.w = w; % Hyperplane defined as w'x+b=0
	    training_data(i).hyperplane.b = b;
	    training_data(i).hyperplane.m = m; % Midpoint
	    training_data(i).hyperplane.s = atan(-w(1)/w(2)); % Hyperplane slope angle
	    training_data(i).hyperplane.s_norm = atan(-w(1)/w(2))/(pi/2); % Normalized between [-1, 1]
	    training_data(i).EV_N.state = EV.traj(:,i:i+N);
	    training_data(i).EV_N.input = EV.inputs(:,i:i+N);
	    training_data(i).TV_N.state = [TV.x(i:i+N) TV.y(i:i+N) TV.heading(i:i+N) TV.v(i:i+N)]';
	    training_data(i).TV_N.input = TV.traj(i:i+N,5:6)';
	    
	    last_w = w;
	    last_b = b;
	end
end