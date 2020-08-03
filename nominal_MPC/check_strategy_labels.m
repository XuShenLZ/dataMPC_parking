close all
clear all

%% Get the solution status from csv file
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["exp_num", "problem_type"];
opts.VariableTypes = ["double", "categorical"];
opts = setvaropts(opts, 2, "EmptyFieldRule", "auto");
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
solution_status = readtable('../data/2020-07-22_log.csv', opts);

clear opts

%%
f = figure();
plt_EV_opt = plot([],[]);
plt_TV = plot([], []);
plt_EV_V = plot([],[]);
plt_TV_V = plot([], []);

start = 440;
for j = start:size(solution_status,1)
%     input('Press enter to advance')
    delete(plt_TV)
    delete(plt_EV_opt)
    delete(plt_EV_V)
    delete(plt_TV_V)

    exp_num = j;
    exp_file = strcat('../data/exp_num_', num2str(exp_num), '.mat');
    load(exp_file)
    
    fprintf('Inspecting experiment %i\n', exp_num)
    
    N = 10; % Prediction horizon
    T = length(TV.t); % Length of data

    if solution_status.problem_type(exp_num) == 'Infeasible'
        fprintf('Solution in file was infeasible, skipping label check\n')
        continue
    end

    min_x_diff = inf;
    min_x_idx = -1;
    rel_y = 0;
    for i = 1:T-N
        % Get x, y, heading, and velocity from ego vehicle at current timestep
        EV_x = EV.traj(1,i);
        EV_y = EV.traj(2,i);
        EV_th = EV.traj(3,i);
        EV_v = EV.traj(4,i);

        EV_curr = [EV_x; EV_y; EV_th; EV_v*cos(EV_th); EV_v*sin(EV_th)];

        % Get x, y, heading, and velocity from target vehicle over prediction
        % horizon
        TV_x = TV.x(i:i+N);
        TV_y = TV.y(i:i+N);
        TV_th = TV.heading(i:i+N);
        TV_v = TV.v(i:i+N);

        TV_pred = [TV_x, TV_y, TV_th, TV_v.*cos(TV_th), TV_v.*sin(TV_th)]';

        % Get target vehicle trajectory relative to ego vehicle state
        rel_state = TV_pred - EV_curr;

        if abs(EV_x - TV_x(1)) < min_x_diff
            min_x_diff = abs(EV_x - TV_x(1));
            min_x_idx = i;
            rel_y = TV_y(1) - EV_y;
        end
    end

    if solution_status.problem_type(exp_num) == 'Emergency Break'
        label = 'yield';
    elseif solution_status.problem_type(exp_num) == 'Successful Maneuver' || solution_status.problem_type(exp_num) == 'Collision Free'
        if rel_y > 0
            label = 'right';
        else
            label = 'left';
        end
    end
    
    EV_x = EV.traj(1,:);
    EV_y = EV.traj(2,:);
    EV_th = EV.traj(3,:);
    EV_v = EV.traj(4,:);
    % Get ego vehicle vertices at current time step
    EV_Vx_curr = [EV_x(min_x_idx) + EV.length/2*cos(EV_th(min_x_idx)) - EV.width/2*sin(EV_th(min_x_idx));
		  EV_x(min_x_idx) + EV.length/2*cos(EV_th(min_x_idx)) + EV.width/2*sin(EV_th(min_x_idx));
		  EV_x(min_x_idx) - EV.length/2*cos(EV_th(min_x_idx)) + EV.width/2*sin(EV_th(min_x_idx));
		  EV_x(min_x_idx) - EV.length/2*cos(EV_th(min_x_idx)) - EV.width/2*sin(EV_th(min_x_idx))];
    EV_Vy_curr = [EV_y(min_x_idx) + EV.length/2*sin(EV_th(min_x_idx)) + EV.width/2*cos(EV_th(min_x_idx));
		  EV_y(min_x_idx) + EV.length/2*sin(EV_th(min_x_idx)) - EV.width/2*cos(EV_th(min_x_idx));
		  EV_y(min_x_idx) - EV.length/2*sin(EV_th(min_x_idx)) - EV.width/2*cos(EV_th(min_x_idx));
		  EV_y(min_x_idx) - EV.length/2*sin(EV_th(min_x_idx)) + EV.width/2*cos(EV_th(min_x_idx))];
    
    TV_x = TV.x;
    TV_y = TV.y;
    TV_th = TV.heading;
    TV_v = TV.v;
    % Get target vehicle vertices at current time step
    TV_Vx_curr = [TV_x(min_x_idx) + TV.length/2*cos(TV_th(min_x_idx)) - TV.width/2*sin(TV_th(min_x_idx));
		  TV_x(min_x_idx) + TV.length/2*cos(TV_th(min_x_idx)) + TV.width/2*sin(TV_th(min_x_idx));
		  TV_x(min_x_idx) - TV.length/2*cos(TV_th(min_x_idx)) + TV.width/2*sin(TV_th(min_x_idx));
		  TV_x(min_x_idx) - TV.length/2*cos(TV_th(min_x_idx)) - TV.width/2*sin(TV_th(min_x_idx))];

	TV_Vy_curr = [TV_y(min_x_idx) + TV.length/2*sin(TV_th(min_x_idx)) + TV.width/2*cos(TV_th(min_x_idx));
		  TV_y(min_x_idx) + TV.length/2*sin(TV_th(min_x_idx)) - TV.width/2*cos(TV_th(min_x_idx));
		  TV_y(min_x_idx) - TV.length/2*sin(TV_th(min_x_idx)) - TV.width/2*cos(TV_th(min_x_idx));
		  TV_y(min_x_idx) - TV.length/2*sin(TV_th(min_x_idx)) + TV.width/2*cos(TV_th(min_x_idx))];

    plt_EV_opt = plot(EV.traj(1, :), EV.traj(2, :), 'b');
    hold on
    plt_TV = plot(TV.x, TV.y, 'r');
    plt_EV_V = plot(EV_Vx_curr, EV_Vy_curr, 'bo');
    plt_TV_V = plot(TV_Vx_curr, TV_Vy_curr, 'ro');
    title(sprintf('exp: %i, label: %s', j, label))
    legend('EV', 'TV')
    map_dim = [-30 30 -10 10];
    axis(map_dim);
    axis equal
end