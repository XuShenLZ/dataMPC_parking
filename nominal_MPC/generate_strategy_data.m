close all
clear all

% uiopen('load')
exp_num = 4;
exp_file = strcat('../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

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

%% Run through data to see which maneuver was executed. 

N = 10; % Prediction horizon
T = length(TV.t); % Length of data

if solution_status.problem_type(exp_num) == 'Infeasible'
    error('Solution in file was infeasible, skipping data generation')
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
    
    training_data(i).X = rel_state;
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

for i = 1:length(training_data)
    training_data(i).Y = label;
end