clear all
close all
clc

pathsetup();

%% Load testing data
% uiopen('load')
exp_num = 1;
exp_file = strcat('../../data/exp_num_', num2str(exp_num), '.mat');
load(exp_file)

%% Load strategy prediction model
model_name = 'nn_strategy_TF-trainscg_h-40_AC-tansig_ep2000_CE0.17453_2020-08-04_15-42';
model_file = strcat('../../models/', model_name, '.mat');
load(model_file)

% Console output saving
if ~isfolder('../data/')
    mkdir('../data')
end

time = datestr(now,'yyyy-mm-dd_HH-MM');
diary(sprintf('../data/FP_NaiveOBCA_Exp%d_%s.txt', exp_num, time))

%%
N = 50; % Prediction horizon
dt = EV.dt; % Time step
T = length(TV.t); % Length of data
v_ref = EV.ref_v; % Reference velocity
y_ref = EV.ref_y; % Reference y
r = sqrt(EV.width^2 + EV.length^2)/2; % Collision buffer radius

n_z = 4;
n_u = 2;

% Instantiate ego vehicle car dynamics
L_r = EV.L/2;
L_f = EV.L/2;
M = 10; % RK4 steps
EV_dynamics = bike_dynamics_rk4(L_r, L_f, dt, M);

% Instantiate obca controller
Q = diag([1 0.1 0.1 0.5]);
R = diag([0.01 0.01]);
d_min = 0.001;
u_u = [0.35; 1];
u_l = [-0.35; -1];
du_u = [0.3; 3];
du_l = [-0.3; -3];

ws_params.name = 'FP_ws_solver_naive';
ws_params.N = N;
ws_params.n_x = n_z;
ws_params.n_u = n_u;
ws_params.n_obs = 1;
ws_params.n_ineq = 4;
ws_params.d_ineq = 2;
ws_params.G = EV.G;
ws_params.g = EV.g;

opt_params.name = 'FP_opt_solver_naive';
opt_params.N = N;
opt_params.n_x = n_z;
opt_params.n_u = n_u;
opt_params.n_obs = 1;
opt_params.n_ineq = 4;
opt_params.d_ineq = 2;
opt_params.G = EV.G;
opt_params.g = EV.g;
opt_params.d_min = d_min;
opt_params.Q = Q;
opt_params.R = R;
opt_params.u_u = u_u;
opt_params.u_l = u_l;
opt_params.du_u = du_u;
opt_params.du_l = du_l;
opt_params.dynamics = EV_dynamics;
opt_params.dt = dt;

obca_controller = obca_controller_FP(true, ws_params, opt_params);

% Instantiate safety controller
d_lim = [-0.35, 0.35];
a_lim = [-1, 1];
du_lim = [0.3; 3];
obca_safety_control = safety_controller(dt, d_lim, a_lim, du_lim);
obca_ebrake_control = ebrake_controller(dt, d_lim, a_lim);

% ==== Filter Setup
V = 0.01 * eye(3);
W = 0.5 * eye(3);
Pm = 0.2 * eye(3);
score = ones(3, 1) / 3;

% Make a copy of EV as the optimal EV, and naive EV
OEV = EV;

obca_mpc_safety = false;

% Data to save
z_traj = zeros(n_z, T-N+1);
z_traj(:,1) = EV.traj(:,1);
u_traj = zeros(n_u, T-N);

z_preds = zeros(n_z, N+1, T-N);
u_preds = zeros(n_u, N, T-N);
z_refs = zeros(n_z, N+1, T-N);

ws_stats = cell(T-N, 1);
sol_stats = cell(T-N, 1);

collide = zeros(T-N, 1);
ebrake = zeros(T-N, 1);

exp_params.exp_num = exp_num;
exp_params.T = T;
exp_params.model = model_name;
exp_params.controller.N = N;
exp_params.controller.Q = Q;
exp_params.controller.R = R;
exp_params.controller.d_min = d_min;
exp_params.controller.d_lim = d_lim;
exp_params.controller.a_lim = a_lim;
exp_params.dynamics.dt = dt;
exp_params.dynamics.M = M;
exp_params.dynamics.L_r = L_r;
exp_params.dynamics.L_f = L_f;
exp_params.dynamics.n_z = n_z;
exp_params.dynamics.n_u = n_u;
exp_params.filter.V = V;
exp_params.filter.W = W;
exp_params.filter.Pm = Pm;

ws_solve_times = zeros(T-N, 1);
opt_solve_times = zeros(T-N, 1);

total_times = zeros(T-N, 1);

%% 
for i = 1:T-N
    tic
    fprintf('\n=================== Iteration: %i ==================\n', i)
    
    % Get x, y, heading, and velocity from ego vehicle at current timestep
    EV_x  = z_traj(1,i);
    EV_y  = z_traj(2,i);
    EV_th = z_traj(3,i);
    EV_v  = z_traj(4,i);

    EV_curr = [EV_x; EV_y; EV_th; EV_v*cos(EV_th); EV_v*sin(EV_th)];
    
    TV_pred = zeros(5, N+1);
    TV_pred(2,:) = 0.3;
    TV_pred(3,:) = 15*pi/180;
    TV.x(i) = 0;
    TV.y(i) = 0.3;
    TV.heading(i) = 15*pi/180;
    TV.v(i) = 0;
    
    z_ref = [z_traj(1,i)+[0:N]*dt*v_ref; zeros(1,N+1); zeros(1,N+1); v_ref*ones(1,N+1)]; 
    
    % Generate target vehicle obstacle descriptions
    tv_obs = get_car_poly_obs(TV_pred, TV.width, TV.length);

    % HPP OBCA MPC
    % Initialize prediction guess for warm start
    if i == 1
        z_ws = z_ref;
        u_ws = zeros(n_u, N);
        u_prev = zeros(n_u, 1);
    else
%         z_ws = z_preds(:,:,i-1);
%         u_ws = u_preds(:,:,i-1);
        z_ws = [z_preds(:,2:end,i-1) EV_dynamics.f_dt(z_preds(:,end,i-1), u_preds(:,end,i-1))];
        u_ws = [u_preds(:,2:end,i-1) u_preds(:,end,i-1)];
        u_prev = u_traj(:,i-1);
    end
    
    fprintf('------- Solving Naive OBCA -------\n')
    
    % Naive controller
    obca_mpc_ebrake = false;
    status_sol = [];
    [status_ws, obca_controller] = obca_controller.solve_ws(z_ws, u_ws, tv_obs);
    if status_ws.success
        ws_solve_times(i) = status_ws.solve_time;
        [z_pred, u_pred, status_sol, obca_controller] = obca_controller.solve(z_traj(:,i), u_prev, z_ref, tv_obs);
    end
    
    if ~status_ws.success || ~status_sol.success
        % If OBCA MPC is infeasible, activate ebrake controller
        obca_mpc_ebrake = true;
        fprintf('Naive HOBCA not feasible, activating emergency brake\n')
    else
        opt_solve_times(i) = status_sol.solve_time;
    end
    
    if obca_mpc_ebrake
        [u_ebrake, obca_ebrake_control] = obca_ebrake_control.solve(z_traj(:,i), TV_pred, u_prev);
        % Assume ebrake control is applied for one time step then no
        % control action is applied for rest of horizon
        u_pred = [u_ebrake zeros(n_u, N-1)];
        z_pred = [z_traj(:,i) zeros(n_z, N)];
        % Simulate this policy
        for j = 1:N
            z_pred(:,j+1) = EV_dynamics.f_dt(z_pred(:,j), u_pred(:,j));
        end
        fprintf('Applying ebrake control\n')
    end
    
    % Simulate system forward using first predicted input
    z_traj(:,i+1) = EV_dynamics.f_dt(z_traj(:,i), u_pred(:,1));
    u_traj(:,i) = u_pred(:,1);
    
    total_times(i) = toc;
    
    % Save data
    z_preds(:,:,i) = z_pred;
    u_preds(:,:,i) = u_pred;
    
    z_refs(:,:,i) = z_ref;

    % Check the collision at the current time step
    collide(i) = check_collision_poly(z_traj(1:3, i), TV_pred(1:3, 1), EV);
    ebrake(i) = obca_mpc_ebrake;

    ws_stats{i} = status_ws;
    sol_stats{i} = status_sol;
end

filename = sprintf('../data/FP_NaiveOBCA_Exp%d_%s.mat', exp_num, time);
save(filename, 'exp_params', 'OEV', 'TV', ...
    'z_traj', 'u_traj', 'z_preds', 'u_preds', 'z_refs', 'ws_stats', 'sol_stats', 'collide', 'ebrake', ...
    'ws_solve_times', 'opt_solve_times', 'total_times')

diary off